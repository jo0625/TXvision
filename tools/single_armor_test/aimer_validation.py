#!/usr/bin/env python3
"""Isolated validation entry. Uses the existing dedicated executor and source snapshot with single-plate geometry support."""
from pathlib import Path
import os,sys,time,json,csv,threading,hashlib
from collections import Counter
B=Path(__file__).resolve().parent
sys.path.insert(0,str(B/'source_snapshot'))
import rclpy
from rclpy.executors import SingleThreadedExecutor,MultiThreadedExecutor,ExternalShutdownException
import basic_aimer_node as original

def stamp(header):return header.stamp.sec*1_000_000_000+header.stamp.nanosec

class Recorder:
 def __init__(self,path,capacity=500000):
  self.path=Path(path) if path else None;self.rows=[];self.counts=Counter();self.lock=threading.Lock();self.capacity=capacity;self.active=0;self.max_active=0
 def emit(self,event,**fields):
  with self.lock:
   if event=='target_begin':self.active+=1;self.max_active=max(self.max_active,self.active)
   if event=='target_end':self.active-=1
   self.counts[event]+=1
   row=dict(event=event,host_ns=time.monotonic_ns(),thread_id=threading.get_native_id(),**fields)
   if len(self.rows)<self.capacity:self.rows.append(row)
   else:self.counts['log_dropped']+=1
   return row
 def matching(self,event,source_stamp_ns):
  with self.lock:return [r for r in self.rows if r['event']==event and r.get('source_stamp_ns')==source_stamp_ns]
 def close(self):
  if not self.path:return
  self.path.parent.mkdir(parents=True,exist_ok=True)
  fields=list(dict.fromkeys(k for r in self.rows for k in r))
  with self.path.open('x',newline='') as f:
   w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(self.rows)
  self.path.with_suffix('.metadata.json').write_text(json.dumps(dict(clock='time.monotonic_ns',unit='ns',resolution_s=time.get_clock_info('monotonic').resolution,counts=dict(self.counts),target_callbacks_max_concurrent=self.max_active,target_callbacks_still_active=self.active),indent=2))

BaseListener=original.TransformListener

def node_class(rec, topology='shared'):
 if rec is None and topology=='shared':return original.BasicAimerNode
 class ObservedListener(BaseListener):
  def __init__(self,buffer,node):
   self.validation_closed=False
   self.validation_tf_node=None
   if topology=='dedicated':
    from rclpy.parameter import Parameter
    self.validation_tf_node=rclpy.create_node('aimer_tf_receiver_validation', context=node.context, namespace=node.get_namespace(),use_global_arguments=False,enable_rosout=False,start_parameter_services=False,parameter_overrides=[Parameter('use_sim_time',value=node.get_parameter('use_sim_time').value)])
    super().__init__(buffer,self.validation_tf_node)
    self.validation_executor=SingleThreadedExecutor(context=node.context)
    self.validation_executor.add_node(self.validation_tf_node)
    def spin_tf():
     try:self.validation_executor.spin()
     except ExternalShutdownException:pass
    self.validation_thread=threading.Thread(target=spin_tf,name='aimer_tf_receiver')
    self.validation_thread.start()
   else:super().__init__(buffer,node)
  def validation_close(self):
   if self.validation_closed:return
   if self.validation_tf_node is not None:
    if not self.validation_executor.shutdown(timeout_sec=5):raise RuntimeError('TF executor did not drain')
    self.validation_thread.join(timeout=5)
    if self.validation_thread.is_alive():raise RuntimeError('TF thread did not stop')
   self.unregister()
   for name in ('tf_sub','tf_static_sub'):
    if hasattr(self,name):delattr(self,name)
   if self.validation_tf_node is not None:self.validation_tf_node.destroy_node()
   self.validation_closed=True
  def __del__(self):
   try:self.validation_close()
   except Exception:pass

  def callback(self,msg):
   for t in msg.transforms:
    if rec:rec.emit('tf_receive_begin',source_stamp_ns=stamp(t.header),source_frame=t.header.frame_id,target_frame=t.child_frame_id)
   try:return super().callback(msg)
   finally:
    for t in msg.transforms:
     if rec:rec.emit('tf_receive_end',source_stamp_ns=stamp(t.header),source_frame=t.header.frame_id,target_frame=t.child_frame_id)
 original.TransformListener=ObservedListener
 class ObservedAimer(original.BasicAimerNode):
  def __init__(self,*a,**kw):
   self.validation_target_stamp=None
   super().__init__(*a,**kw)
   if rec is None:return
   pub=self.publisher;owner=self
   class Publisher:
    def publish(self,msg):
     rec.emit('command_publish',source_stamp_ns=owner.validation_target_stamp,message_stamp_ns=stamp(msg.header),valid=bool(msg.valid),yaw=msg.yaw,pitch=msg.pitch)
     return pub.publish(msg)
   self.publisher=Publisher()
  def target_callback(self,target):
   if rec is None:return super().target_callback(target)
   self.validation_target_stamp=stamp(target.header)
   rec.emit('target_begin',source_stamp_ns=self.validation_target_stamp,source_frame=target.header.frame_id)
   try:return super().target_callback(target)
   finally:
    rec.emit('target_end',source_stamp_ns=self.validation_target_stamp)
    self.validation_target_stamp=None
  def transform_point(self,point,target,destination):
   if rec is None:return super().transform_point(point,target,destination)
   st=stamp(target.header);rec.emit('transform_begin',source_stamp_ns=st,source_frame=target.header.frame_id,target_frame=destination)
   try:
    result=super().transform_point(point,target,destination)
    rec.emit('transform_end',source_stamp_ns=st,result='success');return result
   except BaseException as e:
    rec.emit('transform_end',source_stamp_ns=st,result='error',reason=str(e));raise
  def destroy_node(self):
   self.tf_listener.validation_close()
   return super().destroy_node()
 return ObservedAimer

def executor_for(threads):
 if threads==1:return SingleThreadedExecutor()
 if threads==2:return MultiThreadedExecutor(num_threads=2)
 raise ValueError('Only single-thread control and two-thread candidate are permitted')

def main():
 threads=int(os.environ['AIMER_VALIDATION_THREADS']);out=Path(os.environ['AIMER_VALIDATION_DIRECTORY']);logging=os.environ.get('AIMER_VALIDATION_BOUNDARIES','1')=='1';topology=os.environ.get('AIMER_VALIDATION_TOPOLOGY','shared')
 rclpy.init();rec=Recorder(out/'aimer_boundaries.csv') if logging else None;node=None;executor=None;clean=False
 try:
  node=node_class(rec,topology)();executor=executor_for(threads);executor.add_node(node)
  import tf2_ros.buffer,tf2_ros.transform_listener
  from rclpy import executors as installed_executors
  paths=[Path(original.__file__),Path(tf2_ros.buffer.__file__),Path(tf2_ros.transform_listener.__file__),Path(installed_executors.__file__)]
  identity=dict(tf_topology=topology,executor=type(executor).__name__,threads=threads,instrumentation=logging,pid=os.getpid(),tf_timeout_s=node.tf_timeout,prediction_time_s=node.prediction_time,aim_frame=node.aim_frame,target_callback_group=type(node.default_callback_group).__name__,tf_callback_group=type(node.tf_listener.group).__name__,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
  (out/'aimer_runtime_identity.json').write_text(json.dumps(identity,indent=2))
  (out/'aimer_runtime_maps.txt').write_text(Path('/proc/self/maps').read_text())
  executor.spin()
 except (KeyboardInterrupt,ExternalShutdownException):pass
 finally:
  if executor is not None:
   clean=executor.shutdown(timeout_sec=5)
   if not clean:raise RuntimeError('Executor did not drain; refusing to destroy node while callbacks run')
  if rec:rec.emit('executor_drained',success=clean)
  if node is not None:node.destroy_node()
  if rec:rec.emit('node_destroyed');rec.close()
  (out/'aimer_shutdown.json').write_text(json.dumps(dict(executor_drained=clean,node_destroyed=True,threads=threads)))
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
