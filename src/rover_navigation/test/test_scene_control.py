from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from rover_navigation.scene_control import SceneControl, handler, gateway_handler


class SceneControlTests(unittest.TestCase):
    def setUp(self):
        self.folder = TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        (self.root/'scripts').mkdir()
        (self.root/'scripts/halo_navigation.sh').write_text('launcher')
        (self.root/'navigation_sessions/old').mkdir(parents=True)
        (self.root/'bundle/scenes/scene_050000').mkdir(parents=True)
        (self.root/'bundle/scenes/scene_050000/scene.json').write_text('{}')
        config = dict(scene='/scenes/scenes/scene_050000/scene.json', camera=dict(tilt_rad=.4),
                      database='/previous/map.db', gui='true', rviz='true', fixture='',
                      detector_model='/existing/model.pt', navigation_speed=.1)
        (self.root/'navigation_sessions/old/session.json').write_text(json.dumps(config))
        self.manager = SceneControl(self.root, self.root/'bundle', 'old')

    def test_restart_uses_new_session_without_reusing_the_previous_database(self):
        with patch('rover_navigation.scene_control.threading.Thread'):
            first = self.manager.restart()
            second = self.manager.restart()
        self.assertTrue(first['success'])
        self.assertFalse(second['success'])
        self.assertNotEqual(first['session'], 'old')
        command = self.manager.launch_command(first['session'])
        self.assertIn('mapping_on_confirm:=true', command)
        self.assertIn('detector_model:=/existing/model.pt', command)
        self.assertFalse(any(arg.startswith('database:=') for arg in command))
        self.assertNotIn('fixture:=', command)
        self.assertTrue((self.root/'navigation_sessions/old/session.json').is_file())
        self.assertFalse((self.root/'navigation_sessions'/first['session']).exists())

    def test_supervisor_refuses_to_stop_a_container_from_another_checkout(self):
        inspected = SimpleNamespace(returncode=0, stdout=json.dumps([
            dict(Config=dict(Image='rover-navigation'), Mounts=[dict(Destination='/work', Source='/another-project')])]))
        with patch('rover_navigation.scene_control.subprocess.run', return_value=inspected) as run, \
                patch('rover_navigation.scene_control.urlopen', side_effect=OSError), \
                patch('rover_navigation.scene_control.time.sleep'):
            self.manager.run_restart('new-session')
        self.assertEqual(self.manager.status()['phase'], 'failed')
        self.assertIn('does not belong', self.manager.status()['error'])
        self.assertEqual(run.call_count, 1)
        self.assertEqual(run.call_args.args[0], ['docker', 'container', 'inspect', 'rover-navigation'])

    def test_restart_is_local_and_takes_no_browser_supplied_command(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler(self.manager))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        connection = HTTPConnection('127.0.0.1', server.server_port)
        self.addCleanup(connection.close)
        connection.request('POST', '/restart', headers={'Origin': 'https://elsewhere.invalid'})
        response = connection.getresponse()
        self.assertEqual(response.status, 403)
        response.read()
        connection.request('POST', '/restart', body='command=delete', headers={'Content-Type': 'text/plain'})
        response = connection.getresponse()
        self.assertEqual(response.status, 400)
        response.read()
        self.assertEqual(self.manager.status()['phase'], 'ready')

    def test_restart_page_and_error_remain_available_when_ros_is_down(self):
        self.manager.state.update(phase='failed', error='Scene could not start')
        server = ThreadingHTTPServer(('127.0.0.1', 0), gateway_handler(self.manager))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        connection = HTTPConnection('127.0.0.1', server.server_port)
        self.addCleanup(connection.close)
        with patch('rover_navigation.scene_control.urlopen', side_effect=OSError):
            connection.request('GET', '/status')
            response = connection.getresponse()
            state = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertEqual(state['scene']['error'], 'Scene could not start')
        self.assertTrue(state['scene']['available'])
        connection.request('GET', '/')
        response = connection.getresponse()
        self.assertIn(b'id="restart"', response.read())

    def test_gateway_reuses_control_connection_and_closes_rejected_unread_body(self):
        gateway = gateway_handler(self.manager)
        with patch.object(gateway, 'proxy', return_value=(200, b'{"success":true}', 'application/json', {})) as proxy:
            server = ThreadingHTTPServer(('127.0.0.1', 0), gateway)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            connection = HTTPConnection('127.0.0.1', server.server_port)
            self.addCleanup(connection.close)
            for sequence in (1, 2):
                connection.request('POST', '/drive', body=json.dumps(dict(sequence=sequence)),
                                   headers={'Content-Type': 'application/json'})
                response = connection.getresponse()
                self.assertEqual(response.version, 11)
                self.assertTrue(json.loads(response.read())['success'])
                if sequence == 1:
                    socket = connection.sock
                else:
                    self.assertIs(connection.sock, socket)
            self.assertEqual(proxy.call_count, 2)
            connection.request('POST', '/drive', body='unread', headers={'Origin': 'https://elsewhere.invalid'})
            response = connection.getresponse()
            self.assertEqual(response.status, 403)
            self.assertEqual(response.getheader('Connection'), 'close')
            response.read()
