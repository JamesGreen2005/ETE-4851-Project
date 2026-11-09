import ast
import json
from pathlib import Path
import secrets
import threading
import types
import unittest
import urllib.request
import urllib.error
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from robot_explorer.cameras import CameraFrames


class HeartbeatTests(unittest.TestCase):
    def test_authenticated_heartbeat_is_independent_of_state_render(self):
        tree=ast.parse((Path(__file__).parents[1]/'robot_explorer/dashboard.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        handler=next(n for n in main.body if isinstance(n,ast.ClassDef) and n.name=='Handler')
        events=[]
        node=types.SimpleNamespace(heartbeat=types.SimpleNamespace(publish=lambda m:events.append(m)),
            drive={},exploration={},map_info=None,path=[],robot=None,cameras=CameraFrames())
        namespace=dict(BaseHTTPRequestHandler=BaseHTTPRequestHandler,urlsplit=urlsplit,
            secrets=secrets,json=json,token='test',node=node,Empty=object)
        exec(compile(ast.Module(body=[handler],type_ignores=[]),'<actual dashboard handler>','exec'),namespace)
        server=ThreadingHTTPServer(('127.0.0.1',0),namespace['Handler'])
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)
            with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(base+'/heartbeat')
            self.assertEqual(events,[])
            req=urllib.request.Request(base+'/heartbeat',headers={'X-Control-Token':'test'})
            with urllib.request.urlopen(req) as r:self.assertTrue(json.load(r)['ok'])
            self.assertEqual(len(events),1)
            req=urllib.request.Request(base+'/state',headers={'X-Control-Token':'test'})
            with urllib.request.urlopen(req) as r:json.load(r)
            self.assertEqual(len(events),1)
        finally:server.shutdown();server.server_close();worker.join()
