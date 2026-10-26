"""Host-only restart supervisor for one isolated navigation container."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import subprocess
import threading
import time
from urllib.request import Request, urlopen
from urllib.parse import urlsplit
from importlib.resources import files
from urllib.error import HTTPError
from uuid import uuid4


class SceneControl:
    def __init__(self, project, bundle, session):
        self.project, self.bundle = Path(project).resolve(), Path(bundle).resolve()
        if not re.fullmatch(r'[a-zA-Z0-9_-]+', session):
            raise ValueError('Invalid session name')
        self.sessions = self.project/'navigation_sessions'
        self.config = json.loads((self.sessions/session/'session.json').read_text())
        self.scene = Path(self.config['scene']).parent.name
        if not re.fullmatch(r'scene_[0-9]+', self.scene) or not (self.bundle/'scenes'/self.scene/'scene.json').is_file():
            raise ValueError('Scene is not in the configured bundle')
        if not (self.project/'scripts/halo_navigation.sh').is_file():
            raise ValueError('Navigation launcher missing')
        self.lock = threading.Lock()
        self.state = dict(phase='ready', session=session, error='')
        self.process = None

    def status(self):
        with self.lock:
            return dict(self.state)

    def launch_command(self, session):
        command = ['bash', 'scripts/halo_navigation.sh', str(self.bundle), self.scene, session]
        for key in ('gui', 'rviz', 'fixture', 'terrain_collision', 'navigation_speed',
                    'detector_model', 'detector_backend', 'detector_url', 'terrain_backend', 'terrain_url'):
            if key in self.config and str(self.config[key]):
                command.append(f'{key}:={self.config[key]}')
        command.extend(['preview:=true', 'mission_controls:=true', 'controls_port:=8777', 'mapping_on_confirm:=true'])
        command.append(f"camera_pitch:={self.config['camera'].get('tilt_rad', .4)}")
        # Never copy database:= from the previous session: restart starts fresh.
        return command

    def restart(self):
        with self.lock:
            if self.state['phase'] in ('stopping', 'starting'):
                return dict(success=False, message='A scene restart is already in progress', **self.state)
            session = 'restart-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid4().hex[:6]
            self.state = dict(phase='stopping', session=session, error='')
            result = dict(success=True, message='Restarting scene with a fresh map', **self.state)
        threading.Thread(target=self.run_restart, args=(session,), daemon=True).start()
        return result

    def run_restart(self, session):
        try:
            # Allow the old control page to acknowledge its restart request.
            time.sleep(.5)
            try:
                with urlopen(Request('http://127.0.0.1:8774/stop', data=b'', method='POST'), timeout=4.):
                    pass
            except OSError:
                pass
            inspected = subprocess.run(['docker', 'container', 'inspect', 'rover-navigation'], capture_output=True,
                                       text=True, timeout=10.)
            if inspected.returncode == 0:
                container = json.loads(inspected.stdout)[0]
                mounts = container.get('Mounts', [])
                if (container['Config']['Image'] != 'rover-navigation' or
                        not any(m['Destination'] == '/work' and Path(m['Source']).resolve() == self.project for m in mounts)):
                    raise RuntimeError('Container does not belong to this navigation checkout')
                subprocess.run(['docker', 'stop', '--time', '15', 'rover-navigation'],
                               capture_output=True, check=True, timeout=25.)
                removed = time.monotonic()+10.
                while subprocess.run(['docker', 'container', 'inspect', 'rover-navigation'],
                                     capture_output=True, timeout=5.).returncode == 0:
                    if time.monotonic() >= removed:
                        raise RuntimeError('Previous navigation container has not finished removing')
                    time.sleep(.1)
            logs = self.sessions/'control'
            logs.mkdir(parents=True, exist_ok=True)
            with (logs/(session+'.log')).open('x') as log:
                self.process = subprocess.Popen(self.launch_command(session), cwd=self.project,
                                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            with self.lock:
                self.state['phase'] = 'starting'
            deadline = time.monotonic()+120.
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('Scene launch exited; see navigation_sessions/control/'+session+'.log')
                try:
                    with urlopen('http://127.0.0.1:8774/status', timeout=1.) as response:
                        state = json.load(response)
                    guard = state.get('status', {})
                    if (state.get('session') == session and state.get('mapping', {}).get('waiting_for_confirmation') and
                            guard.get('pose_age_seconds') is not None and guard['pose_age_seconds'] < 1. and
                            guard.get('clock_age_seconds', 99) < .5):
                        with self.lock:
                            self.state['phase'] = 'ready'
                        return
                except (OSError, ValueError):
                    pass
                time.sleep(.5)
            raise RuntimeError('Scene did not finish initializing within 120 seconds; inspect the launch log')
        except Exception as error:
            with self.lock:
                self.state.update(phase='failed', error=str(error))


def handler(manager):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def local(self):
            return (not self.headers.get('Origin') and self.headers.get('Host') in
                    (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'))

        def do_GET(self):
            if not self.local():
                return self.reply(403, dict(error='Local scene control clients only'))
            if self.path == '/status':
                return self.reply(200, manager.status())
            self.reply(404, dict(error='Not found'))

        def do_POST(self):
            if not self.local():
                return self.reply(403, dict(success=False, message='Local scene control clients only'))
            if self.path != '/restart':
                return self.reply(404, dict(success=False, message='Not found'))
            if self.headers.get('Content-Length', '0') != '0':
                return self.reply(400, dict(success=False, message='Restart takes no request body'))
            result = manager.restart()
            self.reply(202 if result['success'] else 409, result)
    return Handler


def gateway_handler(manager):
    """Keep the operator page and restart errors available while ROS restarts."""
    class Gateway(BaseHTTPRequestHandler):
        # Reuse the browser's forwarded TCP channels. Opening a new SSH channel
        # per heartbeat adds avoidable network round trips to the command lease.
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def reply(self, status, data, content='application/json', headers=None):
            body = data if isinstance(data, bytes) else json.dumps(data).encode()
            self.send_response(status)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            if status >= 400:
                # Rejected requests can contain unread bodies. Never reuse that
                # connection as though its next byte began another HTTP request.
                self.close_connection = True
                self.send_header('Connection', 'close')
            for key, value in (headers or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def local(self):
            host, origin = self.headers.get('Host', ''), self.headers.get('Origin')
            try:
                address = urlsplit('http://'+host)
                return (address.hostname in ('127.0.0.1', 'localhost') and address.username is None and
                        0 < (address.port or 80) <= 65535 and (not origin or urlsplit(origin).netloc == host))
            except ValueError:
                return False

        def proxy(self, route, data=None):
            headers = {'Content-Type': self.headers.get('Content-Type', 'application/octet-stream')}
            request = Request('http://127.0.0.1:8777'+route, data=data, headers=headers)
            try:
                response = urlopen(request, timeout=30. if route == '/start' else 4.)
            except HTTPError as error:
                response = error
            with response:
                body = response.read(2*1024*1024)
                extra = {key: response.headers[key] for key in ('X-Image-Stamp', 'X-Annotated') if key in response.headers}
                return response.status, body, response.headers.get('Content-Type', 'application/json'), extra

        def do_GET(self):
            if not self.local():
                return self.reply(403, dict(message='Use the local rover page'))
            route = urlsplit(self.path).path
            if route == '/':
                return self.reply(200, files('rover_navigation').joinpath('mission_controls.html').read_bytes(), 'text/html; charset=utf-8')
            if route not in ('/status', '/camera.jpg', '/map.png', '/diagnostics.png'):
                return self.reply(404, dict(message='Not found'))
            try:
                status, body, content, extra = self.proxy(route)
                if route == '/status':
                    payload = json.loads(body)
                    payload['scene'] = dict(available=True, **manager.status())
                    return self.reply(status, payload)
                return self.reply(status, body, content, extra)
            except (OSError, ValueError):
                if route == '/status':
                    return self.reply(200, dict(scene=dict(available=True, **manager.status())))
                return self.reply(503, dict(message='Scene is initializing'))

        def do_POST(self):
            if not self.local():
                return self.reply(403, dict(success=False, message='Use the local rover page'))
            if self.path == '/restart':
                if self.headers.get('Content-Length', '0') != '0':
                    return self.reply(400, dict(success=False, message='Restart takes no request body'))
                result = manager.restart()
                return self.reply(202 if result['success'] else 409, result)
            if self.path not in ('/drive', '/release', '/stop', '/start', '/manual', '/confirm'):
                return self.reply(404, dict(success=False, message='Not found'))
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 <= size <= 1024 or (self.path not in ('/drive', '/release') and size):
                    raise ValueError('Invalid control body')
                self.connection.settimeout(3.)
                body = self.rfile.read(size)
                if len(body) != size:
                    raise ValueError('Truncated control body')
                return self.reply(*self.proxy(self.path, body))
            except (OSError, ValueError) as error:
                return self.reply(503, dict(success=False, message='Rover control unavailable: '+str(error)))
    return Gateway


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path.cwd())
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--session', required=True, help='Existing session name to adopt')
    args = parser.parse_args()
    manager = SceneControl(args.project, args.bundle, args.session)
    server = ThreadingHTTPServer(('127.0.0.1', 8776), handler(manager))
    server.daemon_threads = True
    gateway = ThreadingHTTPServer(('127.0.0.1', 8774), gateway_handler(manager))
    gateway.daemon_threads = True
    threading.Thread(target=gateway.serve_forever, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        gateway.shutdown()
        gateway.server_close()
        server.server_close()


if __name__ == '__main__':
    main()
