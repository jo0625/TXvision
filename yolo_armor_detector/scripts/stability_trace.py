"""Opt-in bounded observation only; CSV and image writes happen at close()."""
from __future__ import annotations

from collections import Counter
import csv
import json
import math
import os
from pathlib import Path
import time

import numpy as np


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    return value


class BufferedTrace:
    def __init__(self, path, capacity=100000, context=None):
        self.path = Path(path)
        if self.path.exists() or self.path.with_suffix('.metadata.json').exists():
            raise FileExistsError(f'Use a new trace path: {path}')
        if capacity < 1:
            raise ValueError('capacity must be positive')
        self.capacity = capacity
        self.rows = []
        self.counts = Counter()
        self.context = context or {}
        self.closed = False
        self.started_ns = time.monotonic_ns()

    def emit(self, event, *, host_ns=None, **fields):
        self.counts[event] += 1
        if len(self.rows) >= self.capacity:
            self.counts['log_dropped'] += 1
            return None
        row = dict(event=event, host_ns=time.monotonic_ns() if host_ns is None else host_ns,
                   **fields)
        self.rows.append(row)
        return row

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.path.parent.mkdir(parents=True, exist_ok=True)
        keys = list(dict.fromkeys(k for row in self.rows for k in row)) or ['event', 'host_ns']
        with self.path.open('x', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            for row in self.rows:
                writer.writerow({k: json.dumps(plain(v), ensure_ascii=False)
                                 if isinstance(v, (dict, list, tuple, np.ndarray)) else v
                                 for k, v in row.items()})
        metadata = dict(clock='CLOCK_MONOTONIC / time.monotonic_ns', unit='ns',
                        resolution_s=time.get_clock_info('monotonic').resolution,
                        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                        time_namespace=os.readlink('/proc/self/ns/time'),
                        started_ns=self.started_ns, stopped_ns=time.monotonic_ns(),
                        counts=dict(self.counts), context=plain(self.context))
        self.path.with_suffix('.metadata.json').write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')


class DetectorStabilityTrace(BufferedTrace):
    def __init__(self, path, capacity=100000, context=None):
        super().__init__(path, capacity, context)
        self.frame_index = 0
        self.current = {}
        self.candidate = None
        self.image = None
        self.previous = {}
        self.previous_frame = {}
        self.best_pairs = {'corners_px': [], 'pnp_m': [], 'ray_rad': []}

    def begin(self, msg):
        self.frame_index += 1
        self.current = dict(frame_index=self.frame_index, source_stamp_ns=stamp_ns(msg.header),
                            frame_id=msg.header.frame_id, width=msg.width, height=msg.height)
        self.image = None
        self.candidate = None
        self.emit('image', **self.current, encoding=msg.encoding)

    def calibration(self, matrix, distortion):
        self.context['camera_matrix'] = np.array(matrix, copy=True)
        self.context['distortion'] = np.array(distortion, copy=True)

    def candidate_start(self, index, box, confidence, cls, points=None, armor_type='', track_id=''):
        box = np.asarray(box).reshape(4)
        fields = dict(self.current, candidate_index=index, detector_track_id=track_id,
                      identity_kind='box_iou_track' if track_id != '' else 'unavailable',
                      class_id=int(cls), confidence=float(confidence), armor_type=armor_type,
                      box_center_u=float((box[0]+box[2])/2),
                      box_center_v=float((box[1]+box[3])/2),
                      pnp_attempted=False, pnp_success=False, output_valid=False,
                      pnp_point_order='LB,LT,RT,RB', semantic_point_order='LT,LB,RB,RT')
        for name, val in zip(('box_x1','box_y1','box_x2','box_y2'), box):
            fields[name] = float(val)
        if points is not None:
            semantic = np.asarray(points).reshape(4, 2)
            for i, p in enumerate(semantic[[1,0,3,2]]):
                fields[f'p{i}_u'], fields[f'p{i}_v'] = map(float, p)
        self.candidate = self.emit('candidate', **fields)
        return self.candidate

    def pnp(self, success, rvec=None, tvec=None, error=None, failure=''):
        row = self.candidate
        if row is None:
            return
        row.update(pnp_attempted=True, pnp_success=bool(success), failure_reason=failure)
        if rvec is not None:
            for key, v in zip(('rx_rad','ry_rad','rz_rad'), np.asarray(rvec).reshape(3)):
                row[key] = float(v)
        if tvec is not None:
            x, y, z = map(float, np.asarray(tvec).reshape(3))
            row.update(tx_m=x, ty_m=y, tz_m=z, distance_m=math.sqrt(x*x+y*y+z*z),
                       optical_ray_yaw_rad=math.atan2(x,z),
                       optical_ray_pitch_rad=math.atan2(-y,math.hypot(x,z)))
        if error is not None:
            row['reprojection_mean_px'] = float(error)

    def raw_candidate(self, result, confidence, cls):
        pair = result['pair']
        self.candidate_start(result['candidate_index'], result['box_xyxy'], confidence, cls,
                             None if pair is None else pair['points_xy'],
                             result['latched_type'] or '', result['track_id'])
        row = self.candidate
        if row is None:
            return
        row.update(track_created=result['track_created'], ended_track_ids=result['ended_track_ids'],
                   failure_reason=result['failure_reason'])
        pnp = result['pnp']
        if pnp is not None:
            self.pnp(pnp['solved'], pnp.get('rvec'), pnp.get('tvec'),
                     pnp.get('mean_reprojection_error_px'), pnp.get('failure_reason',''))
        # All size hypotheses actually evaluated by production code remain visible.
        dual = result.get('normalized_dual_pnp')
        if dual:
            self.emit('size_hypotheses', **self.current, candidate_index=result['candidate_index'],
                      detector_track_id=result['track_id'], hypotheses=dual)
        row['output_valid'] = bool(result['output_valid'])
        row['failure_reason'] = result['failure_reason']
        self.finish_candidate()

    def finish_candidate(self):
        row = self.candidate
        if row is None:
            return
        key = (row['detector_track_id'], row['class_id'], row['armor_type'], row['frame_id'])
        previous = self.previous.get(key)
        # Unidentified candidates cannot be safely associated across frames.
        if previous is not None and key[0] != '' and row['frame_index'] == previous[0]['frame_index']+1:
            before, before_image = previous
            if row.get('output_valid') and before.get('output_valid'):
                metrics = {}
                if all(f'p{i}_u' in row and f'p{i}_u' in before for i in range(4)):
                    metrics['corners_px'] = max(math.hypot(row[f'p{i}_u']-before[f'p{i}_u'],
                        row[f'p{i}_v']-before[f'p{i}_v']) for i in range(4))
                if 'tx_m' in row and 'tx_m' in before:
                    metrics['pnp_m'] = math.sqrt(sum((row[k]-before[k])**2 for k in ('tx_m','ty_m','tz_m')))
                    metrics['ray_rad'] = max(abs(math.remainder(row[k]-before[k], 2*math.pi))
                        for k in ('optical_ray_yaw_rad','optical_ray_pitch_rad'))
                for metric, score in metrics.items():
                    pairs = self.best_pairs[metric]
                    if len(pairs)<2 or score>min(p[0] for p in pairs):
                        pairs.append((score, dict(before), before_image, dict(row), self.image))
                        pairs.sort(key=lambda p:p[0], reverse=True)
                        del pairs[2:]
        self.previous_frame[key] = (row, self.image)

    def end(self, **fields):
        before_ids = set(self.previous)
        after_ids = set(self.previous_frame)
        if before_ids != after_ids:
            self.emit('identity_set_changed', **self.current,
                      before_identities=sorted(before_ids, key=str),
                      after_identities=sorted(after_ids, key=str))
        self.emit('frame_end', **self.current, **fields)
        self.previous = self.previous_frame
        self.previous_frame = {}
        self.image = None

    def close(self):
        if self.closed:
            return
        import cv2
        folder = self.path.parent / (self.path.stem + '_examples')
        folder.mkdir(parents=True, exist_ok=True)
        examples = []
        for metric, pairs in self.best_pairs.items():
            for rank, (score, before, before_image, after, after_image) in enumerate(pairs, 1):
                examples.append(dict(metric=metric, rank=rank, score=score,
                                     before_stamp_ns=before['source_stamp_ns'],
                                     after_stamp_ns=after['source_stamp_ns']))
                for label, row, image in [('before',before,before_image),('after',after,after_image)]:
                    if image is None:
                        continue
                    prefix = folder / f'{metric}_{rank}_{label}_{row["source_stamp_ns"]}'
                    if not cv2.imwrite(str(prefix)+'.png', image):
                        self.counts['image_write_failures'] += 1
                    overlay = image.copy()
                    for i in range(4):
                        if f'p{i}_u' in row:
                            p = (round(row[f'p{i}_u']), round(row[f'p{i}_v']))
                            cv2.circle(overlay,p,4,(0,255,255),-1)
                            cv2.putText(overlay,str(i),p,cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                    if not cv2.imwrite(str(prefix)+'_corners.png', overlay):
                        self.counts['image_write_failures'] += 1
        self.context.update(examples=examples, example_selection='largest adjacent changes, not acceptance thresholds',
                            unavailable_fields=['hardware armor identity / digit classification'],
                            sample_memory='owned decoded BGR arrays retained, never released SDK pointers')
        super().close()
