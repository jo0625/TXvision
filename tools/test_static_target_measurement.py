"""Scientific/logging invariants: wrapping, association, lost logs, unchanged PnP."""
import csv
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'yolo_armor_detector/scripts'))
from analyze_static_target import distribution, unique_index, clock_identity
from stability_trace import BufferedTrace, DetectorStabilityTrace


def test_wrapped_angle_has_small_span():
    s=distribution(np.deg2rad([179.9,-179.9]),[0,1],angular=True)
    assert s['range']==pytest.approx(.2)
    assert s['std']==pytest.approx(.1)
    assert s['adjacent_abs_max']==pytest.approx(.2)


def test_missing_frame_is_not_an_adjacent_derivative():
    s=distribution([1,100],[0,2],adjacent=[False])
    assert s['n']==2 and s['range']==99
    assert s['adjacent_pairs']==0 and s['adjacent_abs_max'] is None


def test_duplicates_and_clock_namespaces_are_not_silently_matched():
    rows=[dict(stamp=3,value=1),dict(stamp=3,value=2)]
    assert len(unique_index(rows,'stamp')[3])==2
    assert clock_identity(dict(boot_id='a',time_namespace='x')) != clock_identity(
        dict(boot_id='a',time_namespace='y'))


def test_bounded_log_reports_lost_records(tmp_path):
    trace=BufferedTrace(tmp_path/'bounded.csv',capacity=2)
    for i in range(4):
        trace.emit('sample',value=i)
    assert not trace.path.exists()
    trace.close()
    assert len(list(csv.DictReader(trace.path.open())))==2
    metadata=json.loads(trace.path.with_suffix('.metadata.json').read_text())
    assert metadata['counts']['sample']==4
    assert metadata['counts']['log_dropped']==2
    with pytest.raises(FileExistsError):
        BufferedTrace(trace.path)


def test_corners_use_solver_order_and_retained_images_are_unchanged(tmp_path):
    trace=DetectorStabilityTrace(tmp_path/'detector.csv')
    pixels=np.zeros((20,20,3),dtype=np.uint8)
    for frame,x in [(1,0),(2,1)]:
        msg=NS(header=NS(stamp=NS(sec=1,nanosec=frame),frame_id='optical'),
               width=20,height=20,encoding='bgr8')
        trace.begin(msg)
        trace.image=pixels
        points=np.array([[2+x,2],[2+x,10],[10+x,10],[10+x,2]],dtype=np.float32)
        trace.candidate_start(0,[0,0,15,15],.9,0,points,'small',1)
        trace.pnp(True,np.zeros(3),np.array([x*.001,0.,1.]),.2)
        trace.candidate['output_valid']=True
        trace.finish_candidate()
        trace.end(armors_count=1)
    row=next(r for r in trace.rows if r['event']=='candidate')
    assert [row[f'p{i}_v'] for i in range(4)]==[10,2,2,10]
    trace.close()
    assert not pixels.any()  # Annotating examples never changes solver image data.
    assert list((tmp_path/'detector_examples').glob('*before*_corners.png'))
    assert len(list((tmp_path/'detector_examples').glob('*.png')))<=24


def test_enabled_direct_pnp_trace_leaves_output_identical(tmp_path):
    import cv2
    from yolo_armor_detector_node import YoloArmorNode
    from raw_track_latch import _object_points
    node=YoloArmorNode.__new__(YoloArmorNode)
    node.camera_matrix=np.array([[1700.,0,720],[0,1700.,540],[0,0,1]])
    node.dist_coeffs=np.zeros(5)
    node.armor_type='small'
    node.armor_number='armor'
    node.min_pnp_distance=.1
    node.max_pnp_distance=10.
    node.max_reprojection_error=12.
    node.latency_trace=None
    node.stability_trace=None
    points,_=cv2.projectPoints(_object_points('small'),np.array([0.,1.5,0.]),
                              np.array([.1,.05,2.]),node.camera_matrix,node.dist_coeffs)
    semantic=points.reshape(4,2)[[1,0,3,2]]
    baseline=node.solve_armor(semantic)
    trace=DetectorStabilityTrace(tmp_path/'direct.csv')
    trace.current=dict(frame_index=1,source_stamp_ns=1,frame_id='optical',width=1440,height=1080)
    trace.candidate_start(0,[0,0,30,30],.9,0,semantic,'small')
    node.stability_trace=trace
    observed=node.solve_armor(semantic)
    assert observed==baseline
    assert trace.candidate['pnp_success']
    assert trace.candidate['reprojection_mean_px']>=0


def test_pnp_failure_remains_in_log(tmp_path):
    trace=DetectorStabilityTrace(tmp_path/'failed.csv')
    trace.current=dict(frame_index=1,source_stamp_ns=1,frame_id='optical',width=10,height=10)
    trace.candidate_start(0,[0,0,9,9],.8,0,None,'small',2)
    trace.pnp(False,failure='solver_error')
    assert trace.candidate['pnp_attempted'] is True
    assert trace.candidate['pnp_success'] is False
    assert trace.candidate['failure_reason']=='solver_error'
