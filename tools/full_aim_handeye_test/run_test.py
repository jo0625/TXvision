#!/usr/bin/env python3
"""Current-device preflight, then previously authorized bounded real-image test."""
from pathlib import Path
import os,sys,csv,json,subprocess,signal,hashlib,math
B=Path(__file__).resolve().parent
SETTINGS=json.loads((B/'runtime_paths.json').read_text())
W=Path(SETTINGS['dependency_root']);results=[];active=None
from check_config import check_config

def read(path):
 if not path.exists():return []
 out=[]
 for r in csv.DictReader(path.open()):
  d={}
  for k,v in r.items():
   if v in ('',None):d[k]=None
   elif v in ('True','False'):d[k]=v=='True'
   else:
    try:d[k]=json.loads(v)
    except (ValueError,TypeError):d[k]=v
  out.append(d)
 return out

def check_ready():
 if not os.access('/dev/ttyACM0',os.R_OK|os.W_OK):raise RuntimeError('Serial unavailable or permission missing')
 check_config(verbose=False)
 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:names=[Path(a.decode(errors='replace')).name for a in p.read_bytes().split(b'\0')[:3]]
  except OSError:continue
  if any(n in ('serial_bridge_node.py','dynamic_follow_test.py','aimer_validation.py','hik_camera_node','armor_tracker_node') for n in names):raise RuntimeError('Pipeline already running: '+p.parent.name+' '+str(names))

def run(name,duration,motion=False):
 global active
 check_ready()
 cmd=['bash',str(W/'outputs/tf_deadlock_fix/with_fix.sh'),'/usr/bin/env','ROS_LOG_DIR='+str(B/'ros_logs'),'PYTHONDONTWRITEBYTECODE=1','/usr/bin/python3','-B',str(B/'capture.py'),'--output',str(B/name),'--expected-tf2',str(W/'work/tf_deadlock/install/tf2/lib/libtf2.so'),'--warmup','7','--duration',str(duration),'--prediction-time','0.0','--boundaries']
 cmd += ['--motion','--static-gate',str(B/'static_gate.json')] if motion else ['--receive-only']
 print(json.dumps(dict(event='starting',run=name,motion=motion,duration_s=duration)),flush=True)
 with (B/(name+'.console.log')).open('x') as log:
  active=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
  try:rc=active.wait(timeout=duration+70)
  except subprocess.TimeoutExpired:
   active.send_signal(signal.SIGINT)
   try:rc=active.wait(timeout=35)
   except subprocess.TimeoutExpired:active.terminate();rc=active.wait(timeout=8)
  active=None
 cap=json.loads((B/name/'capture.json').read_text()) if (B/name/'capture.json').exists() else {}
 item=dict(run_id=name,returncode=rc,complete=bool(cap.get('windows') and cap['windows'][0]['complete']),error=cap.get('error'),process_exit_codes=cap.get('process_exit_codes'),command=cmd);results.append(item);(B/'runs.json').write_text(json.dumps(results,indent=2));print(json.dumps(dict(event='finished',**{k:v for k,v in item.items() if k!='command'})),flush=True)
 if rc or not item['complete'] or cap.get('error'):raise RuntimeError('Capture incomplete or failed: '+name)
 return cap

try:
 cap=run('preflight_01',15)
 w=cap['windows'][0];lo,hi=w['start_ns'],w['end_ns'];window=lambda r:lo<=r['host_ns']<=hi
 rows=[r for r in read(B/'preflight_01/pipeline.csv') if window(r)];arm=[r for r in rows if r['event']=='armors' and r['armor_count']>0];pv=[r for r in rows if r['event']=='preview_command' and r['valid']];fb=[r for r in rows if r['event']=='feedback'];aim=read(B/'preflight_01/aimer.csv');fail=[r for r in aim if r['event']=='aimer_failure' and window(r)];bounds=json.loads((B/'preflight_01/aimer_boundaries.metadata.json').read_text());t=[lo]+[r['host_ns'] for r in pv]+[hi];maxgap=max(b-a for a,b in zip(t,t[1:]))/1e9
 checks=dict(valid_armors_at_least_100=len(arm)>=100,valid_preview_at_least_100=len(pv)>=100,preview_maxgap_below_half_second=maxgap<.5,no_aimer_failures=not fail,feedback_vision_mode=bool(fb) and all(r['mode']==6 for r in fb),feedback_fresh_at_window_end=bool(fb) and hi-fb[-1]['host_ns']<150_000_000,no_boundary_log_drops=bounds['counts'].get('log_dropped',0)==0,callbacks_serialized=bounds['target_callbacks_max_concurrent']==1)
 gate=dict(motion_permitted=all(checks.values()),checks=checks,source_run='preflight_01',formal_duration_s=w['duration_s'],nonempty_armors=len(arm),valid_preview=len(pv),aimer_failures=len(fail),max_valid_preview_gap_s=maxgap,scope='Current 30s readiness; prior dedicated TF regression and 3x120s static evidence retained separately',prediction_time=0)
 (B/'static_gate.json').write_text(json.dumps(gate,indent=2));print(json.dumps(dict(event='preflight_gate',**gate)),flush=True)
 if not gate['motion_permitted']:raise RuntimeError('Preflight readiness checks failed; motion not started')
 run('motion_01',60,True)
except BaseException as exc:
 if active is not None and active.poll() is None:
  active.send_signal(signal.SIGINT)
  try:active.wait(timeout=35)
  except subprocess.TimeoutExpired:active.terminate();active.wait(timeout=8)
 (B/'runner_error.json').write_text(json.dumps(dict(error=repr(exc)),indent=2));print(repr(exc),flush=True);raise
finally:
 (B/'runs.json').write_text(json.dumps(results,indent=2))
