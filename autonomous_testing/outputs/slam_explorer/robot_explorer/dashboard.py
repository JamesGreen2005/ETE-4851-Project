"""Local-only map/camera dashboard and operator supervision heartbeat."""

import json

import math

import secrets

import threading

import time

from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

from urllib.parse import urlsplit

import rclpy

from rclpy.node import Node

from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data

from rclpy.time import Time

from std_msgs.msg import String,Empty

from std_srvs.srv import SetBool

from sensor_msgs.msg import Image,LaserScan

from nav_msgs.msg import OccupancyGrid,Path

from tf2_ros import Buffer,TransformListener

from .cameras import CameraFrames



PAGE=r'''<!doctype html><html><head><meta charset="utf-8"><title>Robot mapping</title>

<meta name="viewport" content="width=device-width,initial-scale=1"><style>

:root{color-scheme:dark}*{box-sizing:border-box}body{font:15px system-ui;background:#0d1520;color:#e7eef7;margin:0;padding:20px;max-width:1800px;margin-inline:auto}h1{font-size:26px;margin:0}h2{font-size:18px;margin:0 0 12px}p{color:#aabbd0;line-height:1.5}header{display:flex;gap:16px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin-bottom:14px}.controls{display:flex;gap:8px;flex-wrap:wrap}button{padding:12px 20px;border:1px solid #42546a;border-radius:10px;background:#26374b;color:#fff;font:600 16px system-ui;cursor:pointer}button:disabled{opacity:.45;cursor:wait}#start{background:#167551;border-color:#29946c}#stop{background:#bd343f;border-color:#e35a67}#reset{background:#294a73}#stopReason{color:#fff;margin:0}#actionMessage:empty{display:none}.status-bar{background:#182739;border:1px solid #30475f;border-radius:12px;padding:12px 16px;margin-bottom:16px}.feeds{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px;margin-bottom:16px}.card{background:#142131;border:1px solid #2c4057;border-radius:14px;padding:16px;min-width:0}.feed-head{display:flex;justify-content:space-between;gap:10px;align-items:center}.camera-state{color:#ffcf70;font-size:13px;margin:0 0 12px}.camera-state.live{color:#94e6bd}.feed-screen{aspect-ratio:4/3;background:#080f19;border-radius:9px;overflow:hidden;display:flex;align-items:center;justify-content:center}img{width:100%;height:100%;object-fit:contain}img[hidden]{display:none}canvas{max-width:100%}#lidar{width:100%;height:100%}.readings{margin-bottom:16px}.table-wrap{overflow-x:auto}table{width:100%;border-collapse:collapse;text-align:left;white-space:nowrap}th{color:#91a8c2;font-size:12px;text-transform:uppercase;letter-spacing:.04em}th,td{padding:12px 14px;border-bottom:1px solid #283b52}td{font-variant-numeric:tabular-nums}#sensorFreshness{margin-top:0}#sensorSummary{font:14px system-ui;line-height:1.8;color:#c0d0e2}#steeringSummary{margin-bottom:0}.map-card{margin-bottom:16px}#map{display:block;max-height:540px;margin:auto;image-rendering:pixelated;background:#637081}pre{white-space:pre-wrap;overflow-wrap:anywhere}details{background:#111c2a;border:1px solid #2c4057;border-radius:12px;padding:16px;margin-top:12px}summary{cursor:pointer;color:#aabbd0}details h3{font-size:14px;color:#aabbd0}details pre{font:12px ui-monospace,monospace;color:#8098b2}.hint{font-size:12px;margin:8px 0 0}@media(max-width:1000px){.feeds{grid-template-columns:repeat(2,minmax(0,1fr))}.lidar-card{grid-column:span 2}.lidar-card .feed-screen{aspect-ratio:2/1}}@media(max-width:620px){body{padding:12px}.feeds{grid-template-columns:1fr}.lidar-card{grid-column:auto}.lidar-card .feed-screen{aspect-ratio:4/3}button{padding:12px 14px}.controls{width:100%}.controls button{flex:1}.card{padding:12px}}

</style></head>

<body><header><div><h1>Explore and map</h1><p class="hint">Keep this dashboard connected while moving.</p></div><div class="controls"><button id="start">Start exploration</button><button id="stop">STOP</button><button id="reset">Reset navigation</button></div></header>

<div class="status-bar"><p id="stopReason" role="status">Connecting to robot…</p><p id="actionMessage" role="status"></p></div>

<section class="feeds" aria-label="Live camera and LiDAR feeds">

<article class="card"><div class="feed-head"><h2>Front camera</h2><p id="frontStatus" class="camera-state" role="status">Waiting</p></div><div class="feed-screen"><img id="camera" alt="Front camera live view" hidden></div></article>

<article class="card"><div class="feed-head"><h2>Rear camera</h2><p id="rearStatus" class="camera-state" role="status">Waiting</p></div><div class="feed-screen"><img id="rearCamera" alt="Rear camera live view" hidden></div></article>

<article class="card lidar-card"><div class="feed-head"><h2>Live LiDAR</h2><p id="lidarStatus" class="camera-state" role="status">Waiting</p></div><div class="feed-screen"><canvas id="lidar" width="560" height="420" aria-label="Live LiDAR returns in robot coordinates"></canvas></div><p class="hint">Robot centred · forward is up · rings every metre</p></article>

</section>

<section class="card readings"><h2>Live motor and sensor readings</h2><p id="sensorFreshness">Waiting for telemetry</p><div class="table-wrap">

<table><thead><tr><th>Motor</th><th>Current (A)</th><th>Voltage (V)</th><th>Power (W)</th><th>Encoder count</th><th>Counts/s</th><th>PWM (%)</th><th>Current sensor</th></tr></thead>

<tbody><tr><td>1</td><td id="m1current">—</td><td id="m1voltage">—</td><td id="m1power">—</td><td id="m1count">—</td><td id="m1rate">—</td><td id="m1duty">—</td><td id="m1sensor">Waiting</td></tr>

<tr><td>2</td><td id="m2current">—</td><td id="m2voltage">—</td><td id="m2power">—</td><td id="m2count">—</td><td id="m2rate">—</td><td id="m2duty">—</td><td id="m2sensor">Waiting</td></tr></tbody></table></div>

<p id="steeringSummary"></p><pre id="sensorSummary"></pre></section>

<section class="card map-card"><h2>Map and planned route</h2><canvas id="map" aria-label="SLAM map and planned route"></canvas></section>

<details id="diagnostics"><summary>Technical details and raw diagnostics</summary><p id="shuntNote"></p><h3>Navigation and sensor diagnostics</h3><pre id="status">Connecting…</pre><h3>Raw encoder and board telemetry</h3><pre id="telemetry"></pre></details>

<script>

const token=new URLSearchParams(location.hash.slice(1)).get('token')||'';

const headers={'X-Control-Token':token};let busy=false;

const number=(v,digits=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(digits):'—';

function renderSensors(s){const drive=s.drive||{},data=drive.sensors||{},age=s.drive_age_s;const fresh=data.fresh===true&&typeof age==='number'&&age<=1.5;document.getElementById('sensorFreshness').textContent=fresh?'Live board telemetry · age '+number(data.age_s+age)+' s':'Telemetry unavailable or stale';for(let i=1;i<=2;i++){const motor=(data.motors||[]).find(m=>m.motor===i)||{};for(const [key,field,digits] of [['current','current_a',3],['voltage','voltage_v',2],['power','power_w',2],['count','count',0],['rate','counts_per_s',0],['duty','duty_pct',1]])document.getElementById('m'+i+key).textContent=fresh?number(motor[field],digits):'—';document.getElementById('m'+i+'sensor').textContent=fresh?(motor.current_status||'unavailable'):'stale';}document.getElementById('shuntNote').textContent='Current conversion uses '+number(data.ina_shunt_ohms,3)+' ohm shunts; match ina_shunt_ohms in calibration to your boards.';document.getElementById('steeringSummary').textContent='Steering: 1000–2000 µs · straight 1500 µs · front/rear target '+(fresh?(drive.steering_pulses||[]).join(' / '):'—');const f=fresh?(drive.fields||{}):{},ages=drive.sensor_ages||{},coverage=drive.rear_coverage||{};document.getElementById('sensorSummary').textContent='Heading / roll / pitch: '+(f.H??'—')+' / '+(f.R??'—')+' / '+(f.P??'—')+' degrees\nIMU valid: '+(f.VALID??'—')+' · calibration: '+(f.CAL??'—')+'\nLiDAR / IMU / EKF age: '+['lidar','imu','ekf'].map(k=>typeof age==='number'&&typeof ages[k]==='number'?number(ages[k]+age)+' s':'—').join(' / ')+'\nFront / rear video age: '+['front','rear'].map(k=>number(s.cameras?.[k]?.age_s)+' s').join(' / ')+'\nSpeed: '+(fresh?number(drive.speed,3):'—')+' m/s · odometry encoder: '+(drive.encoder_index??'—')+'\nRear coverage: '+(coverage.known_bins??'—')+'/'+(coverage.required_bins??'—')+' sectors (minimum 1)'+ '\nReverse output time: '+number(drive.reverse_budget?.active_seconds)+' / 16 s · distance: '+number(drive.reverse_budget?.distance_m)+' / 2.15 m'+(drive.reverse_budget?.last_limit?'\nLast reverse limit: '+drive.reverse_budget.last_limit.cause+' at '+number(drive.reverse_budget.last_limit.active_seconds)+' s / '+number(drive.reverse_budget.last_limit.distance_m)+' m':'');const exploration=s.exploration||{};document.getElementById('start').disabled=Boolean(resetBusy||exploration.resetting||exploration.cancelling_actions);document.getElementById('reset').disabled=resetBusy;document.getElementById('stopReason').textContent=exploration.enabled?'Exploring · '+(exploration.reason||''):'Stopped: '+(exploration.reason||drive.reason||'Waiting')+(exploration.last_stop?' · last stop: '+exploration.last_stop.reason:'');}

async function call(path,body,timeoutMs=4000){let r=await fetch(path,{method:body?'POST':'GET',headers:{...headers,...(body?{'Content-Type':'application/json'}:{})},body:body?JSON.stringify(body):undefined,signal:AbortSignal.timeout(timeoutMs)});if(!r.ok)throw Error(await r.text());return r;}

async function action(enable){try{let r=await call('/action',{enable});const result=await r.json();document.getElementById('actionMessage').textContent=result.message||(result.success?'Request accepted':'Request refused')}catch(e){document.getElementById('actionMessage').textContent=e.message}}

let resetBusy=false;

async function resetNavigation(){if(resetBusy)return;resetBusy=true;document.getElementById('reset').disabled=true;document.getElementById('start').disabled=true;try{const r=await call('/reset',{reset:true},10000);const result=await r.json();document.getElementById('actionMessage').textContent=result.message||'Reset requested; wait for Ready, then press Start'}catch(e){document.getElementById('actionMessage').textContent='Reset failed: '+e.message}finally{resetBusy=false;document.getElementById('reset').disabled=false}}

let hbBusy=false;setInterval(async()=>{if(hbBusy)return;hbBusy=true;try{await fetch('/heartbeat',{headers,signal:AbortSignal.timeout(700)})}catch(e){}finally{hbBusy=false}},250);

document.querySelector('#start').onclick=()=>action(true);document.querySelector('#stop').onclick=()=>action(false);document.getElementById('reset').onclick=resetNavigation;

document.addEventListener('keydown',e=>{if(e.code==='Space'||e.key==='Escape'){e.preventDefault();action(false)}});

setInterval(async()=>{if(busy)return;busy=true;try{let s=await(await call('/state')).json();renderSensors(s);renderLidar(s.lidar);document.querySelector('#status').textContent=JSON.stringify({drive:{state:s.drive.state,reason:s.drive.reason,health:s.drive.health,speed:s.drive.speed,requested_speed:s.drive.requested_speed,applied_duty:s.drive.applied_duty,reverse_budget:s.drive.reverse_budget,obstacle:s.drive.obstacle,rear_coverage:s.drive.rear_coverage},exploration:s.exploration,fusion:s.fusion,motor_ceiling:s.drive.max_duty_pct,wheel_confidence:s.drive.wheel_confidence},null,2);document.querySelector('#telemetry').textContent=JSON.stringify(s.drive.fields||{},null,2);const m=s.map;if(m){const c=document.querySelector('#map');c.width=m.width;c.height=m.height;const ctx=c.getContext('2d');let img=await createImageBitmap(await(await call('/map.png')).blob());ctx.drawImage(img,0,0);let xy=p=>[(p[0]-m.x)/m.resolution,m.height-(p[1]-m.y)/m.resolution];if(s.path.length){ctx.strokeStyle='#08f';ctx.lineWidth=2;ctx.beginPath();s.path.forEach((p,i)=>{let q=xy(p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.stroke()}if(s.robot){let p=xy(s.robot);ctx.fillStyle='#f22';ctx.beginPath();ctx.arc(...p,4,0,7);ctx.fill()}}}catch(e){renderSensors({});renderLidar(null);document.querySelector('#status').textContent=e.message}finally{busy=false}},500);

function renderLidar(scan){const c=document.getElementById('lidar'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);const cx=c.width/2,cy=c.height/2,scale=Math.min(c.width,c.height)/8;ctx.strokeStyle='#25394e';ctx.lineWidth=1;for(let r=1;r<=4;r++){ctx.beginPath();ctx.arc(cx,cy,r*scale,0,Math.PI*2);ctx.stroke()}ctx.beginPath();ctx.moveTo(cx,0);ctx.lineTo(cx,c.height);ctx.moveTo(0,cy);ctx.lineTo(c.width,cy);ctx.stroke();const status=document.getElementById('lidarStatus');const fresh=scan&&scan.fresh===true;status.textContent=fresh?'Live · '+scan.points.length+' returns':'No fresh scan';status.classList[fresh?'add':'remove']('live');if(fresh){ctx.fillStyle='#69dbc7';for(const [x,y] of scan.points){if(Math.abs(x)<=4&&Math.abs(y)<=4)ctx.fillRect(cx-y*scale-1.5,cy-x*scale-1.5,3,3)}}ctx.fillStyle='#78aaff';ctx.beginPath();ctx.moveTo(cx,cy-10);ctx.lineTo(cx-7,cy+7);ctx.lineTo(cx+7,cy+7);ctx.closePath();ctx.fill();}

const views={front:{id:'camera',status:'frontStatus',path:'/camera.jpg'},rear:{id:'rearCamera',status:'rearStatus',path:'/camera/rear.jpg'}};

for(const view of Object.values(views)){view.busy=false;view.url=null;view.goodAt=0;}

function clearCamera(view,message){const el=document.getElementById(view.id);el.hidden=true;el.removeAttribute('src');if(view.url)URL.revokeObjectURL(view.url);view.url=null;view.goodAt=0;const status=document.getElementById(view.status);status.textContent=message;status.classList.remove('live');}

async function camera(name){const view=views[name];if(view.busy||document.hidden)return;view.busy=true;try{const r=await call(view.path,undefined,800);const blob=await r.blob();if(document.hidden)return;const url=URL.createObjectURL(blob);const el=document.getElementById(view.id);el.src=url;el.hidden=false;if(view.url)URL.revokeObjectURL(view.url);view.url=url;view.goodAt=performance.now();const status=document.getElementById(view.status);status.textContent='Live';status.classList.add('live');}catch(e){clearCamera(view,'No fresh '+name+' camera image');}finally{view.busy=false;}}

for(const [name,view] of Object.entries(views))document.getElementById(view.id).onerror=()=>clearCamera(view,'Unable to display '+name+' camera image');



setInterval(()=>{for(const [name,view] of Object.entries(views)){if(view.goodAt&&performance.now()-view.goodAt>1200)clearCamera(view,'Stale '+name+' camera image');camera(name);}},350);

document.addEventListener('visibilitychange',()=>{if(document.hidden)for(const view of Object.values(views))clearCamera(view,'Camera paused');});

</script></body></html>'''





class Dashboard(Node):

    def __init__(self):

        super().__init__('exploration_dashboard')

        import cv2

        import numpy as np

        from cv_bridge import CvBridge

        self.cv2=cv2;self.np=np;self.cv=CvBridge();self.drive={};self.exploration={};self.fusion={};self.drive_received=None

        self.map_info=None;self.png=b'';self.path=[];self.robot=None;self.lidar_scan=None

        self.cameras=CameraFrames();self.camera_encoded={'front':0.,'rear':0.}

        self.declare_parameter('front_camera_topic','/camera/image_raw')

        self.declare_parameter('rear_camera_topic','/camera/rear/image_raw')

        self.heartbeat=self.create_publisher(Empty,'operator_heartbeat',10)

        self.explore_client=self.create_client(SetBool,'explore/enable')

        self.drive_client=self.create_client(SetBool,'drive/enable')

        self.reset_client=self.create_client(SetBool,'explore/reset')

        self.control_lock=threading.Lock()

        self.tf=Buffer();self.listener=TransformListener(self.tf,self)

        self.create_subscription(String,'drive/status',self.drive_cb,10)

        self.create_subscription(String,'fusion/status',lambda m:setattr(self,'fusion',json.loads(m.data)),10)

        self.create_subscription(String,'explore/status',lambda m:setattr(self,'exploration',json.loads(m.data)),10)

        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid,'navigation/map',self.map_cb,qos)

        self.create_subscription(LaserScan,'scan',self.scan_cb,qos_profile_sensor_data)

        self.create_subscription(Path,'plan',lambda m:setattr(self,'path',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)

        for name in ('front','rear'):

            self.create_subscription(Image,self.get_parameter(name+'_camera_topic').value,

                lambda msg,name=name:self.camera_cb(msg,name),qos_profile_sensor_data)

        self.create_timer(.5,self.pose_cb)



    def drive_cb(self,msg):

        self.drive=json.loads(msg.data);self.drive_received=time.monotonic()



    def map_cb(self,msg):

        if msg.info.width*msg.info.height>4000000:return

        a=self.np.array(msg.data,dtype=self.np.int16).reshape(msg.info.height,msg.info.width)

        image=self.np.where(a<0,128,self.np.where(a>=50,0,255)).astype(self.np.uint8)

        ok,png=self.cv2.imencode('.png',self.np.flipud(image))

        if ok:

            self.png=png.tobytes();self.map_info=dict(width=msg.info.width,height=msg.info.height,

                resolution=msg.info.resolution,x=msg.info.origin.position.x,y=msg.info.origin.position.y)



    def scan_cb(self,msg):

        try:

            now=self.get_clock().now().nanoseconds/1e9

            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

            if not -.1<=now-stamp<=.5:return

            mount=self.tf.lookup_transform('base_link',msg.header.frame_id,Time()).transform

            q=mount.rotation

            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

            points=[]

            for i in range(0,len(msg.ranges),max(1,math.ceil(len(msg.ranges)/720))):

                r=msg.ranges[i]

                if not math.isfinite(r) or not msg.range_min<r<min(msg.range_max,8.):continue

                angle=msg.angle_min+i*msg.angle_increment+yaw

                points.append([mount.translation.x+r*math.cos(angle),mount.translation.y+r*math.sin(angle)])

            self.lidar_scan=dict(points=points,received=time.monotonic(),source_age_s=max(0.,now-stamp))

        except Exception:pass



    def lidar_state(self):

        scan=self.lidar_scan

        age=None if scan is None else time.monotonic()-scan['received']+scan['source_age_s']

        fresh=age is not None and age<=.6

        return dict(fresh=fresh,age_s=age,points=scan['points'] if fresh else [])



    def camera_cb(self,msg,name='front'):

        if time.monotonic()-self.camera_encoded[name]<.3:return

        try:

            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

            now=self.get_clock().now().nanoseconds/1e9

            if stamp<=0 or not -.1<=now-stamp<=1.:return

            image=self.cv.imgmsg_to_cv2(msg,'bgr8')

            ok,jpeg=self.cv2.imencode('.jpg',image,[self.cv2.IMWRITE_JPEG_QUALITY,65])

            if ok:

                self.cameras.update(name,jpeg.tobytes(),stamp,self.get_clock().now().nanoseconds/1e9)

                self.camera_encoded[name]=time.monotonic()

        except Exception:pass



    def pose_cb(self):

        try:

            t=self.tf.lookup_transform('map','base_link',Time()).transform.translation

            self.robot=[t.x,t.y]

        except Exception:self.robot=None



    def service(self,client,value):

        if not client.service_is_ready():raise RuntimeError('Service unavailable')

        req=SetBool.Request();req.data=value;f=client.call_async(req)

        end=time.monotonic()+4

        while not f.done() and time.monotonic()<end:time.sleep(.02)

        if not f.done():raise RuntimeError('Service timed out')

        res=f.result();return dict(success=res.success,message=res.message)





def main():

    rclpy.init();node=Dashboard();token=secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):

        def log_message(self,*args):pass

        def reply(self,status,body,kind='application/json'):

            if not isinstance(body,bytes):body=(json.dumps(body) if kind=='application/json' else body).encode()

            self.send_response(status);self.send_header('Content-Type',kind)

            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers()

            self.wfile.write(body)

        def authorized(self):return secrets.compare_digest(self.headers.get('X-Control-Token',''),token)

        def do_GET(self):

            path=urlsplit(self.path).path

            if path=='/':self.reply(200,PAGE,'text/html');return

            if not self.authorized():self.reply(403,{'error':'Open the complete token link'});return

            if path=='/heartbeat':

                node.heartbeat.publish(Empty())

                self.reply(200,{'ok':True})

            elif path=='/state':

                self.reply(200,dict(lidar=node.lidar_state() if hasattr(node,'lidar_state') else None,fusion=getattr(node,'fusion',{}),drive=node.drive,exploration=node.exploration,map=node.map_info,path=node.path,robot=node.robot,cameras=node.cameras.status(),drive_age_s=(node.cameras.clock()-node.drive_received if getattr(node,'drive_received',None) is not None else None)))

            elif path=='/map.png' and node.png:self.reply(200,node.png,'image/png')

            elif path in ('/camera.jpg','/camera/rear.jpg'):

                jpeg=node.cameras.get('rear' if path=='/camera/rear.jpg' else 'front')

                if jpeg:self.reply(200,jpeg,'image/jpeg')

                else:self.reply(503,{'error':'No fresh camera image'})

            else:self.reply(404,{'error':'No data yet'})

        def do_POST(self):

            if not self.authorized():self.reply(403,{'error':'Unauthorized'});return

            try:

                size=int(self.headers.get('Content-Length','0'))

                if not 0<size<1000:raise ValueError('Invalid request size')

                body=json.loads(self.rfile.read(size))

                with node.control_lock:

                    if self.path=='/reset' and body.get('reset') is True:

                        node.service(node.drive_client,False)

                        result=node.service(node.reset_client,True)

                    elif self.path=='/action' and type(body.get('enable')) is bool:

                        if not body['enable']:node.service(node.drive_client,False)

                        result=node.service(node.explore_client,body['enable'])

                    else:raise ValueError('Invalid action')

                self.reply(200,result)

            except Exception as e:self.reply(400,{'error':str(e)})

    server=ThreadingHTTPServer(('127.0.0.1',8005),Handler)

    worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()

    print('SLAM-EXPLORER-0.17 http://127.0.0.1:8005/#token='+token,flush=True)

    try:rclpy.spin(node)

    except KeyboardInterrupt:pass

    finally:server.shutdown();node.destroy_node();rclpy.shutdown()

