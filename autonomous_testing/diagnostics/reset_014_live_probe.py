import json,re,time,urllib.request,urllib.error
log='/tmp/slam_014_reset_live.log';end=time.monotonic()+48;token=None
while time.monotonic()<end:
    try:
        text=open(log).read();match=re.search(r'SLAM-EXPLORER-0.14 http://127.0.0.1:8005/#token=(\S+)',text)
        if match:token=match.group(1);break
    except OSError:pass
    time.sleep(.2)
assert token,'Dashboard did not start'
base='http://127.0.0.1:8005'
def request(path,body=None,authorized=True):
    headers={'X-Control-Token':token} if authorized else {}
    if body is not None:headers['Content-Type']='application/json'
    req=urllib.request.Request(base+path,data=json.dumps(body).encode() if body is not None else None,headers=headers)
    with urllib.request.urlopen(req,timeout=6) as response:
        data=response.read()
        return json.loads(data) if response.headers.get_content_type()=='application/json' else data.decode()
try:request('/reset',{'reset':True},False);raise AssertionError('Unauthenticated reset accepted')
except urllib.error.HTTPError as exc:assert exc.code==403
assert 'id="reset"' in request('/')
observed=0
for cycle in range(2):
    if cycle:request('/action',{'enable':False})
    request('/heartbeat');result=request('/reset',{'reset':True});assert result['success'],result
    began=time.monotonic()
    while time.monotonic()<end:
        request('/heartbeat');state=request('/state');drive=state['drive'];exploration=state['exploration']
        if drive:
            assert drive['enabled'] is False,drive
            assert drive['state']=='disarmed',drive
            assert all(abs(v)<.01 for v in drive['applied_duty']),drive
            observed+=1
        if time.monotonic()-began>1.2 and not exploration.get('resetting') and 'Reset complete. Ready' in exploration.get('reason',''):
            print('PASS reset cycle',cycle+1,'same token/page, confirmed disarmed and Ready',flush=True);break
        time.sleep(.25)
    else:raise AssertionError('Reset not ready: '+json.dumps(state.get('exploration')))
assert 'id="reset"' in request('/')
request('/action',{'enable':False})
print('PASS authenticated HTTP reset, unauthorized rejection, Stop -> Reset twice; zero motor output across',observed,'status checks',flush=True)
