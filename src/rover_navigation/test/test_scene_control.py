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

    def test_restart_preserves_candidate_image_and_separate_camera_range(self):
        self.manager.config['container_image']='rover-navigation-phase1-opaque-v2'
        self.manager.config['install_relative']='navigation_sessions/rendering-phase1/install'
        self.manager.config['camera'].update(rgb_far_m=20,far_m=8)
        self.manager.config.update(manual_max_linear_speed=.05,manual_max_angular_speed=.3)
        command=self.manager.launch_command('candidate-restart')
        self.assertIn('NAV_IMAGE=rover-navigation-phase1-opaque-v2',command)
        self.assertIn('NAV_INSTALL_REL=navigation_sessions/rendering-phase1/install',command)
        self.assertIn('camera_rgb_far:=20',command)
        self.assertIn('manual_max_linear_speed:=0.05',command)
        self.assertIn('manual_max_angular_speed:=0.3',command)

    def test_v2_restart_and_generation_keep_map_extent_and_conservative_initial_speed(self):
        self.manager.config.update(map_size_m=40.,manual_max_linear_speed=.2,
                                   manual_initial_linear_speed=.05,manual_max_angular_speed=.3,camera_rate_hz=15)
        for generated in (False,True):
            command=self.manager.launch_command('new-session',generated=generated)
            self.assertIn('map_size_m:=40.0',command)
            self.assertIn('manual_initial_linear_speed:=0.05',command)
            self.assertIn('manual_max_linear_speed:=0.2',command)
            self.assertIn('camera_rate_hz:=15',command)
            self.assertNotIn('database:=/previous/map.db',command)

    def test_demo_restart_preserves_profile_and_invalidates_runtime_session(self):
        self.manager.config.update(demo_mode=True,demo_profile='rehearsal',camera_rate_hz=15)
        command=self.manager.launch_command('fresh-demo')
        self.assertIn('demo_mode:=true',command);self.assertIn('demo_profile:=rehearsal',command)
        with patch('rover_navigation.scene_control.threading.Thread'):result=self.manager.restart()
        self.assertNotEqual(result['session'],'old')

    def test_browser_profile_stays_gui_free_after_restart_and_generation(self):
        self.manager.config.update(gui='false',rviz='false',sensor_headless='false',sensor_render_engine='ogre',
                                   camera_rate_hz=15,map_size_m=40.,demo_mode=True,demo_profile='rehearsal')
        for generated in (False,True):
            command=self.manager.launch_command('browser-demo',generated=generated)
            for value in ('gui:=false','rviz:=false','sensor_headless:=false','sensor_render_engine:=ogre',
                          'camera_rate_hz:=15','map_size_m:=40.0','demo_mode:=true'):
                self.assertIn(value,command)
            self.assertFalse(any(arg.startswith('database:=') for arg in command))

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

    def test_new_scene_requires_assets_and_serializes_requests(self):
        self.assertFalse(self.manager.restart(new_scene=True)['success'])
        assets = self.root/'bundle/assets'
        assets.mkdir()
        (assets/'catalog.json').write_text('[]')
        with patch('rover_navigation.scene_control.threading.Thread') as worker:
            result = self.manager.restart(new_scene=True)
            self.assertTrue(result['success'])
            self.assertTrue(result['session'].startswith('generated-'))
            self.assertEqual(worker.call_args.kwargs['args'], (result['session'], True))
            self.assertFalse(self.manager.restart()['success'])
        self.assertEqual((self.root/'bundle/scenes/scene_050000/scene.json').read_text(), '{}')

    def test_generation_isolated_from_source_assets_and_fresh_launch_keeps_renderer(self):
        assets = self.root/'bundle/assets'
        assets.mkdir()
        (assets/'catalog.json').write_text('[]')
        (assets/'mesh.dae').write_text('original mesh')
        self.manager.config.update(fixture='flat', sensor_render_engine='ogre', sensor_headless='false')
        self.manager.config['container_image']='sha256:'+'1'*64

        def generated(args, **kwargs):
            destination = self.root/'navigation_sessions/generated-scenes/new-scene/scenes/scene_1000017'
            destination.mkdir(parents=True)
            (destination/'scene.json').write_text('{}')
            self.assertIn('/work:ro', ' '.join(args))
            self.assertIn('rover_navigation.scene_generation', args)
            self.assertIn('sha256:'+'1'*64,args)

        with patch('rover_navigation.scene_control.secrets.randbelow', return_value=17), \
                patch('rover_navigation.scene_control.os.getuid', return_value=1001, create=True), \
                patch('rover_navigation.scene_control.os.getgid', return_value=1001, create=True), \
                patch('rover_navigation.scene_control.subprocess.run', side_effect=generated):
            bundle, scene = self.manager.generate_scene('new-scene')
        self.assertEqual((bundle/'assets/mesh.dae').read_text(), 'original mesh')
        self.assertEqual((assets/'mesh.dae').read_text(), 'original mesh')
        command = self.manager.launch_command('new-scene', bundle, scene, generated=True)
        self.assertNotIn('fixture:=flat', command)
        self.assertIn('sensor_render_engine:=ogre', command)
        self.assertIn('sensor_headless:=false', command)
        self.assertIn('mapping_on_confirm:=true', command)
        self.assertEqual(self.manager.bundle, (self.root/'bundle').resolve())

    def test_v2_generator_uses_separate_source_tree_and_bounded_material_time(self):
        assets=self.root/'bundle/assets';assets.mkdir();(assets/'catalog.json').write_text('[]')
        (self.root/'bundle/scenes/scene_050000/scene.json').write_text(json.dumps(dict(config=dict(terrain=dict(profile_version=2)))))
        module=self.root/'source/src/rover_navigation/rover_navigation/scene_generation.py'
        module.parent.mkdir(parents=True);module.write_text('qualified source')
        def generated(args,**kwargs):
            self.assertIn('PYTHONPATH=/work/source/src/rover_navigation:/work/source/src/mars_training_environment:/work/source/src/rover_training',args)
            self.assertEqual(kwargs['timeout'],600.)
            dest=self.root/'navigation_sessions/generated-scenes/v2-generation/scenes/scene_1000017'
            dest.mkdir(parents=True);(dest/'scene.json').write_text('{}')
        with patch('rover_navigation.scene_control.secrets.randbelow',return_value=17), \
                patch('rover_navigation.scene_control.os.getuid',return_value=1001,create=True), \
                patch('rover_navigation.scene_control.os.getgid',return_value=1001,create=True), \
                patch('rover_navigation.scene_control.subprocess.run',side_effect=generated):
            self.manager.generate_scene('v2-generation')

    def test_gateway_new_scene_is_local_and_accepts_no_parameters(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), gateway_handler(self.manager))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with patch.object(self.manager, 'restart', return_value=dict(success=True, session='new', phase='stopping')) as restart:
            for origin, body, expected in (('https://elsewhere.invalid', '', 403), (None, 'seed=2', 400), (None, '', 202)):
                connection = HTTPConnection('127.0.0.1', server.server_port)
                headers = {'Origin': origin} if origin else {}
                connection.request('POST', '/new-scene', body=body, headers=headers)
                response = connection.getresponse()
                self.assertEqual(response.status, expected)
                response.read()
                connection.close()
            restart.assert_called_once_with(new_scene=True)

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
