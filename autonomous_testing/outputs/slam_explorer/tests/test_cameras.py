import ast
import json
from pathlib import Path
import secrets
import threading
import tempfile
import types
import unittest
import urllib.error
import urllib.request
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot_explorer.cameras import CameraFrames


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.frames = CameraFrames(clock=lambda: self.now)

    def test_separate_feeds_disconnect_and_reconnect(self):
        self.assertIsNone(self.frames.get('rear'))
        self.frames.update('front', b'front', 100., 100.)
        self.frames.update('rear', b'rear', 100., 100.)
        self.assertEqual(self.frames.get('front'), b'front')
        self.assertEqual(self.frames.get('rear'), b'rear')
        self.now += 1.1
        self.frames.update('front', b'new front', 101.1, 101.1)
        self.assertIsNone(self.frames.get('rear'))
        self.assertTrue(self.frames.status()['front']['fresh'])
        self.assertEqual(self.frames.status()['rear']['state'], 'stale')
        self.frames.update('rear', b'reconnected', 101.2, 101.2)
        self.assertEqual(self.frames.get('rear'), b'reconnected')

    def test_old_future_invalid_and_repeated_source_frames_are_rejected(self):
        for stamp, now in [(0., 100.), (98., 100.), (101., 100.), (float('nan'), 100.)]:
            self.assertFalse(self.frames.update('rear', b'bad', stamp, now))
        self.frames.update('rear', b'original', 100., 100.)
        self.now += .8
        self.assertFalse(self.frames.update('rear', b'repeated', 100., 100.8))
        self.now += .3
        self.assertIsNone(self.frames.get('rear'))

    def test_source_delay_counts_towards_expiration(self):
        self.frames.update('rear', b'delayed', 99.2, 100.)
        self.now += .3
        self.assertIsNone(self.frames.get('rear'))

    def test_http_routes_authenticate_and_never_return_stale_video(self):
        tree = ast.parse((Path(__file__).parents[1]/'robot_explorer/dashboard.py').read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        handler = next(n for n in main.body if isinstance(n, ast.ClassDef) and n.name == 'Handler')
        events = []
        node = types.SimpleNamespace(cameras=self.frames, drive={}, exploration={}, map_info=None,
            path=[], robot=None, heartbeat=types.SimpleNamespace(publish=lambda m: events.append(m)))
        namespace = dict(BaseHTTPRequestHandler=BaseHTTPRequestHandler, urlsplit=urlsplit,
                         secrets=secrets, json=json, token='test', node=node, Empty=object)
        exec(compile(ast.Module(body=[handler], type_ignores=[]), '<dashboard handler>', 'exec'), namespace)
        server = ThreadingHTTPServer(('127.0.0.1', 0), namespace['Handler'])
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = 'http://127.0.0.1:'+str(server.server_port)

        def get(path, token='test'):
            return urllib.request.urlopen(urllib.request.Request(base+path, headers={'X-Control-Token': token}))

        try:
            self.frames.update('front', b'front jpeg', 100., 100.)
            self.frames.update('rear', b'rear jpeg', 100., 100.)
            for path, jpeg in [('/camera.jpg', b'front jpeg'), ('/camera/rear.jpg', b'rear jpeg')]:
                with self.assertRaises(urllib.error.HTTPError) as e:
                    get(path, token='wrong')
                self.assertEqual(e.exception.code, 403)
                with get(path) as r:
                    self.assertEqual(r.headers['Content-Type'], 'image/jpeg')
                    self.assertEqual(r.read(), jpeg)
            self.now += 1.1
            with self.assertRaises(urllib.error.HTTPError) as e:
                get('/camera/rear.jpg')
            self.assertEqual(e.exception.code, 503)
            with get('/state') as r:
                self.assertFalse(json.load(r)['cameras']['rear']['fresh'])
            with get('/heartbeat') as r:
                self.assertTrue(json.load(r)['ok'])
            self.assertEqual(len(events), 1)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    def test_camera_launch_keeps_devices_and_namespaces_separate(self):
        tree = ast.parse((Path(__file__).parents[1]/'launch/cameras.launch.py').read_text())
        setup = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'setup')
        namespace = dict(Path=Path, Node=lambda **kwargs: kwargs,
            LaunchConfiguration=lambda name: types.SimpleNamespace(perform=lambda context: context[name]))
        exec(compile(ast.Module(body=[setup], type_ignores=[]), '<camera launch>', 'exec'), namespace)
        with tempfile.TemporaryDirectory() as directory:
            front = Path(directory)/'front'
            rear = Path(directory)/'rear'
            front.touch()
            rear.touch()
            context = {}
            for role, device in [('front', front), ('rear', rear)]:
                context.update({'start_'+role+'_camera': 'true', role+'_camera_device': str(device),
                    role+'_camera_width': '640', role+'_camera_height': '480',
                    role+'_camera_fps': '15.0', role+'_camera_pixel_format': 'yuyv'})
            nodes = namespace['setup'](context)
            self.assertEqual([n['namespace'] for n in nodes], ['camera', 'camera/rear'])
            self.assertEqual([n['parameters'][0]['video_device'] for n in nodes], [str(front), str(rear)])
            self.assertEqual(nodes[1]['parameters'][0]['frame_id'], 'rear_camera_optical_frame')
            context['front_camera_device'] = str(rear)
            with self.assertRaisesRegex(ValueError, 'different capture devices'):
                namespace['setup'](context)
            context['start_front_camera'] = 'false'
            self.assertEqual(len(namespace['setup'](context)), 1)
            rear.unlink()
            with self.assertRaisesRegex(ValueError, 'verified capture device'):
                namespace['setup'](context)


if __name__ == '__main__':
    unittest.main()
