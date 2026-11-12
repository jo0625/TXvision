#!/usr/bin/env python3
"""Offline static-target statistics; no thresholds, no actuator access."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
import struct
from pathlib import Path

import numpy as np


def cell(text):
    if text in ('',None):
        return None
    if text in ('True','False'):
        return text=='True'
    try:
        return json.loads(text)
    except (ValueError,TypeError):
        return text


def read_csv(path):
    if not path.exists():
        return []
    with path.open(newline='') as stream:
        return [{k:cell(v) for k,v in row.items()} for row in csv.DictReader(stream)]


def write_csv(path, rows, fallback=('event',)):
    fields=list(dict.fromkeys(k for row in rows for k in row)) or list(fallback)
    with path.open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        writer.writerows({k:json.dumps(v) if isinstance(v,(list,dict,tuple)) else v
                          for k,v in row.items()} for row in rows)


def distribution(values, times, *, angular=False, adjacent=None):
    a=np.asarray(values,dtype=float)
    t=np.asarray(times,dtype=float)
    if not len(a):
        return None
    if angular:
        a=np.rad2deg(np.unwrap(a))
    steps=np.abs(np.diff(a))
    if adjacent is not None:
        steps=steps[np.asarray(adjacent,dtype=bool)]
    slope=float(np.polyfit(t-t[0],a-a[0],1)[0]) if len(a)>1 and np.ptp(t)>0 else None
    return dict(n=len(a),std=float(np.std(a)),p5=float(np.percentile(a,5)),
        p95=float(np.percentile(a,95)),p5_p95_span=float(np.percentile(a,95)-np.percentile(a,5)),
        minimum=float(np.min(a)),maximum=float(np.max(a)),range=float(np.ptp(a)),
        adjacent_pairs=len(steps),adjacent_abs_p95=float(np.percentile(steps,95)) if len(steps) else None,
        adjacent_abs_max=float(np.max(steps)) if len(steps) else None,linear_slope_per_s=slope,
        net_first_last=float(a[-1]-a[0]))


def run_at(ns,windows):
    return next((w['run'] for w in windows if w['start_ns']<=ns<w['end_ns']),0)


def unique_index(rows,key):
    index=defaultdict(list)
    for row in rows:
        if row.get(key) is not None:
            index[row[key]].append(row)
    return index


def clock_identity(metadata):
    return metadata.get('boot_id'),metadata.get('time_namespace')


def analyze(folder,output):
    output.mkdir(parents=True,exist_ok=False)
    capture=json.loads((folder/'capture.json').read_text())
    windows=capture['windows']
    datasets={name:read_csv(folder/(name+'.csv')) for name in
              ('camera','detector','pipeline','aimer','preview','serial')}
    import cv2
    for row in datasets['detector']:
        if row.get('pnp_success') and all(isinstance(row.get(k),(int,float)) for k in ('rx_rad','ry_rad','rz_rad')):
            rotation,_=cv2.Rodrigues(np.asarray([row[k] for k in ('rx_rad','ry_rad','rz_rad')]))
            normal=rotation[:,0]  # The armor plane is object x=0; +object-x is its normal.
            row['normal_heading_camera_rad']=math.atan2(normal[0],normal[2])
            row['normal_elevation_camera_rad']=math.atan2(-normal[1],math.hypot(normal[0],normal[2]))
    metas={name:json.loads((folder/(name+'.metadata.json')).read_text())
           for name in datasets if (folder/(name+'.metadata.json')).exists()}
    identity={clock_identity(m) for m in metas.values()}
    clocks_match=len(identity)==1 and all(all(x) for x in identity)
    camera_by_stamp=unique_index(datasets['camera'],'source_stamp_ns')
    candidate_by_stamp=unique_index([r for r in datasets['detector'] if r['event']=='candidate'],
                                    'source_stamp_ns')
    issues=Counter()
    for name,rows in datasets.items():
        for row in rows:
            row['component']=name
            row['run']=run_at(row['host_ns'],windows) if clocks_match else 0
            if row.get('event') in ('run_start','run_end','tf','feedback','preview_feedback'):
                row['camera_link_status']='not_an_image_event'
                continue
            candidates=camera_by_stamp.get(row.get('source_stamp_ns'),[])
            if len(candidates)==1:
                row['sdk_frame_number']=candidates[0]['sdk_frame_number']
                row['source_camera_host_ns']=candidates[0]['host_ns']
                row['camera_link_status']='unique'
            else:
                row['camera_link_status']='ambiguous' if len(candidates)>1 else 'unmatched'
                if row['run'] and row.get('source_stamp_ns') is not None:
                    issues[name+'_'+row['camera_link_status']]+=1
    # Never match by arrival proximity. Unsent and multiply-sent images remain separate.
    sends=[r for r in datasets['serial'] if r.get('event')=='send']
    for row in sends:
        wire=row.get('legacy_wire_hex') or row.get('wire_hex')
        if wire:
            packet=bytes.fromhex(wire)
            if len(packet)==31:
                packet=packet[18:29]
            if len(packet)==11 and packet[0]==0xa5 and packet[-2:]==b'\x5a\x5b':
                row['yaw'],row['pitch']=struct.unpack('<ff',packet[1:9])
                row['angle_source']='decoded actual float32 wire bytes'
    sends_by_stamp=unique_index(sends,'source_stamp_ns')
    solutions=unique_index([r for r in datasets['aimer'] if r['event']=='aimer_solution'],'source_stamp_ns')
    preview=unique_index([r for r in datasets['preview'] if r['event']=='preview_conversion'],'source_stamp_ns')
    joined=[]
    for image in [r for r in datasets['detector'] if r['event']=='image']:
        stamp=image['source_stamp_ns']
        matched=candidate_by_stamp.get(stamp,[])
        for candidate in matched or [None]:
            row=dict(image)
            if candidate:
                row.update(candidate)
            else:
                row.update(event='image_without_candidates',output_valid=False)
            row.update(send_events=len(sends_by_stamp.get(stamp,[])),
                       send_status='not_sent' if not sends_by_stamp.get(stamp) else 'send_events_recorded',
                       aimer_matches=len(solutions.get(stamp,[])),preview_matches=len(preview.get(stamp,[])))
            if len(solutions.get(stamp,[]))==1:
                for k,v in solutions[stamp][0].items():
                    row['aimer_'+k]=v
            if len(preview.get(stamp,[]))==1:
                for k,v in preview[stamp][0].items():
                    row['preview_'+k]=v
            joined.append(row)
    detector_stamps={r['source_stamp_ns'] for r in datasets['detector'] if r['event']=='image'}
    for camera in datasets['camera']:
        if camera['source_stamp_ns'] not in detector_stamps:
            joined.append(dict(camera,event='camera_without_detector_observation',send_events=0,
                send_status='not_sent',output_valid=False))
    write_csv(output/'per_frame.csv',joined)
    write_csv(output/'send_events.csv',sends,('event','source_stamp_ns','call_before_ns','yaw','pitch'))
    groups=defaultdict(list)
    for row in datasets['detector']:
        if row['event']!='candidate' or not row['run']:
            continue
        key=(row['run'],'pnp',row.get('detector_track_id'),row.get('class_id'),
             row.get('armor_type'),row.get('frame_id'),bool(row.get('output_valid')))
        # Missing detector identity is not replaced by candidate array order.
        if row.get('detector_track_id') is None:
            issues['candidates_without_cross_frame_identity']+=1
            key=(*key,row['frame_index'],row.get('candidate_index'))
        groups[key].append(row)
    for name,event in [('aimer','aimer_solution'),('preview','preview_conversion')]:
        for row in datasets[name]:
            if row['event']!=event or not row['run']:
                continue
            matches=[c for c in candidate_by_stamp.get(row.get('source_stamp_ns'),[]) if c.get('output_valid')]
            if len(matches)!=1:
                issues[name+'_ambiguous_candidate_association']+=1
                continue
            c=matches[0]
            aim=solutions.get(row['source_stamp_ns'],[])
            selected=aim[0].get('selected_armor_index') if len(aim)==1 else None
            row['frame_index']=c['frame_index']
            row['detector_track_id']=c['detector_track_id']
            key=(row['run'],name,c['detector_track_id'],c['class_id'],c['armor_type'],row['frame_id'],
                 selected)
            groups[key].append(row)
    metrics=[f'p{i}_{axis}' for i in range(4) for axis in ('u','v')]+[
        'box_center_u','box_center_v','tx_m','ty_m','tz_m','rx_rad','ry_rad','rz_rad',
        'distance_m','reprojection_mean_px','optical_ray_yaw_rad','optical_ray_pitch_rad',
        'normal_heading_camera_rad','normal_elevation_camera_rad',
        'aim_x_m','aim_y_m','aim_z_m','yaw_rad','pitch_rad','feedback_yaw_rad','feedback_pitch_rad']
    stats=[]
    identity_stats=[]
    segments=[]
    for key,rows in groups.items():
        rows.sort(key=lambda r:r['host_ns'])
        for metric in metrics:
            valid=[r for r in rows if isinstance(r.get(metric),(int,float)) and math.isfinite(r[metric])]
            if not valid:
                continue
            angular=metric.endswith('_rad') and metric not in ('rx_rad','ry_rad','rz_rad')
            unit='deg' if angular else ('rad' if metric.endswith('_rad') else 'm' if metric.endswith('_m') else 'px')
            adjacent=[b['frame_index']==a['frame_index']+1 and
                (a.get('sdk_frame_number') is None or b.get('sdk_frame_number') is None or
                 ((b['sdk_frame_number']-a['sdk_frame_number']) & 0xffffffff)==1)
                for a,b in zip(valid,valid[1:])]
            identity_stats.append(dict(group=json.dumps(key,separators=(',',':')),run=key[0],stage=key[1],
                metric=metric,unit=unit,**distribution([r[metric] for r in valid],
                    [r['host_ns']/1e9 for r in valid],angular=angular,adjacent=adjacent)))
        # No derivative or unwrapping across missing frames, identity/type changes or failures.
        parts=[]
        for row in rows:
            if (not parts or row['frame_index']!=parts[-1][-1]['frame_index']+1 or
                (row.get('sdk_frame_number') is not None and parts[-1][-1].get('sdk_frame_number') is not None
                 and ((row['sdk_frame_number']-parts[-1][-1]['sdk_frame_number']) & 0xffffffff)!=1)):
                parts.append([])
            parts[-1].append(row)
        for segment,part in enumerate(parts,1):
            label=json.dumps(key,separators=(',',':'))+f'/segment{segment}'
            segments.append((key,label,part))
            for metric in metrics:
                valid=[r for r in part if isinstance(r.get(metric),(int,float)) and math.isfinite(r[metric])]
                if not valid:
                    continue
                # Rodrigues vector components are not Euler angles; do not wrap them.
                angular=metric.endswith('_rad') and metric not in ('rx_rad','ry_rad','rz_rad')
                unit='deg' if angular else ('rad' if metric.endswith('_rad') else
                    'm' if metric.endswith('_m') else 'px')
                result=distribution([r[metric] for r in valid],[r['host_ns']/1e9 for r in valid],
                    angular=angular,adjacent=[b['frame_index']==a['frame_index']+1 for a,b in zip(valid,valid[1:])])
                stats.append(dict(group=label,run=key[0],stage=key[1],metric=metric,unit=unit,**result))
    write_csv(output/'statistics.csv',stats)
    write_csv(output/'statistics_by_identity.csv',identity_stats)
    write_csv(output/'events.csv',[r for name,rows in datasets.items() for r in rows
        if r.get('event') in ('identity_set_changed','aimer_failure','aimer_invalid','source_rejected',
            'source_overwritten_before_send','source_expired_or_disconnected','serial_error') or
            (r.get('event')=='candidate' and not r.get('output_valid'))])

    # Rank changes, never turn a rank into an arbitrary pass/fail threshold.
    ranked=[]
    for key,label,part in segments:
        if key[1]!='pnp':
            continue
        for before,after in zip(part,part[1:]):
            if all(k in before and k in after and before[k] is not None and after[k] is not None
                   for k in ('tx_m','ty_m','tz_m','p0_u','p0_v','p1_u','p1_v','p2_u','p2_v','p3_u','p3_v')):
                ranked.append(dict(group=label,run=key[0],
                    before_stamp_ns=before['source_stamp_ns'],after_stamp_ns=after['source_stamp_ns'],
                    before_frame_number=before.get('sdk_frame_number'),after_frame_number=after.get('sdk_frame_number'),
                    max_corner_step_px=max(math.hypot(after[f'p{i}_u']-before[f'p{i}_u'],
                        after[f'p{i}_v']-before[f'p{i}_v']) for i in range(4)),
                    position_step_m=math.sqrt(sum((after[k]-before[k])**2 for k in ('tx_m','ty_m','tz_m'))),
                    ray_yaw_step_deg=math.degrees(abs(math.remainder(
                        after['optical_ray_yaw_rad']-before['optical_ray_yaw_rad'],2*math.pi))),
                    ray_pitch_step_deg=math.degrees(abs(math.remainder(
                        after['optical_ray_pitch_rad']-before['optical_ray_pitch_rad'],2*math.pi)))))
    ranked.sort(key=lambda r:r['position_step_m'],reverse=True)
    write_csv(output/'ranked_adjacent_changes.csv',ranked)
    configuration=read_csv(folder/'camera.csv.configuration.csv')
    start_config={r['node']:r for r in configuration if r['phase']=='start'}
    stop_config={r['node']:r for r in configuration if r['phase']=='stop'}
    config_changes=[dict(node=k,start=start_config[k],stop=stop_config.get(k)) for k in start_config
        if start_config[k].get('status')!=0 or k not in stop_config or
           stop_config[k].get('status')!=0 or start_config[k].get('value')!=stop_config[k].get('value')]
    frame_stats=[]
    for w in windows:
        rows=[r for r in datasets['camera'] if w['start_ns']<=r['host_ns']<w['end_ns']]
        deltas=[(b['sdk_frame_number']-a['sdk_frame_number']) & 0xffffffff for a,b in zip(rows,rows[1:])]
        images=[r for r in datasets['detector'] if r['event']=='image' and r['run']==w['run']]
        frames=[r for r in datasets['detector'] if r['event']=='frame_end' and r['run']==w['run']]
        cands=[r for r in datasets['detector'] if r['event']=='candidate' and r['run']==w['run']]
        frame_stats.append(dict(run=w['run'],duration_s=w['duration_s'],camera_frames=len(rows),
            sdk_gaps=sum(d-1 for d in deltas if 1<d<0x80000000),sdk_duplicates=sum(d==0 for d in deltas),
            sdk_counter_resets_or_reverse=sum(d>=0x80000000 for d in deltas),
            detector_images=len(images),camera_received_fps=len(rows)/w['duration_s'],
            camera_frames_without_detector_observation=sum(r['source_stamp_ns'] not in detector_stamps for r in rows),
            detector_fps=len(frames)/w['duration_s'],frames_without_valid_armor=sum(not r.get('armors_count') for r in frames),
            candidates=len(cands),invalid_candidates=sum(not r.get('output_valid') for r in cands),
            pnp_failed_attempts=sum(r.get('pnp_attempted') and not r.get('pnp_success') for r in cands),
            actual_send_events=sum(r['run']==w['run'] for r in sends)))
    write_csv(output/'run_summary.csv',frame_stats)
    summary=dict(capture_error=capture.get('error'),windows=windows,runs=frame_stats,
        clock_identities_match=clocks_match,clock_metadata=metas,
        association_issues=dict(issues),configuration_changes_or_read_errors=config_changes,
        camera_counters={k:v.get('value') for k,v in stop_config.items() if k in ('log_dropped','grab_errors')},
        event_counts={name:dict(Counter(r.get('event','camera_frame') for r in rows)) for name,rows in datasets.items()},
        actual_send_events=len(sends),threshold='No verified project stability acceptance threshold found; no pass/fail judgment',
        coverage=dict(vision=any(r['event']=='candidate' for r in datasets['detector']),
                      feedback=any(r.get('event')=='feedback' for r in datasets['pipeline']),
                      aimer=any(r.get('event')=='aimer_solution' for r in datasets['aimer']),
                      controller_conversion=any(r.get('event')=='preview_conversion' for r in datasets['preview']),
                      actual_sending=bool(sends)),
        findings='Per-stage amplitudes and ranked events are evidence, not proof of a causal mechanism',
        statistics='population standard deviation; linear P5/P95; same identity/class/type/frame/selection/run; adjacent steps exclude SDK/detector gaps; statistics.csv additionally separates contiguous segments')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    make_plots(output,segments,sends,windows)
    report=['# 静止目标稳定性测量','',
        '本报告不使用任意门限宣布正常。是否满足控制要求尚缺验收阈值。',
        '统计按试次、目标 ID、类别、尺寸类型、坐标系、重建装甲板选择和连续段隔离；角度使用最短角差展开，单位为度。',
        'SDK 帧号通过相同图像 header.stamp 精确关联，无法唯一关联的记录保留。',
        '实际发送未启用；发送数为零时只说明本次未测，不能视为发送稳定或下位机已收到。','',
        '| 试次 | 时长 s | SDK 帧数 | 检测帧数 | 无有效装甲板帧 | 无效候选 | SDK 缺口 |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in frame_stats:
        report.append(f'| {r["run"]} | {r["duration_s"]:.3f} | {r["camera_frames"]} | {r["detector_images"]} | {r["frames_without_valid_armor"]} | {r["invalid_candidates"]} | {r["sdk_gaps"]} |')
    report+=['','同身份每试次统计在 statistics_by_identity.csv，分连续段结果在 statistics.csv，逐帧关联在 per_frame.csv，全部发送事件在 send_events.csv，失败/切换在 events.csv。',
        'ranked_adjacent_changes.csv 将同段相邻 PnP 变化按位移排序，同时列出角点变化，便于检查 A/B 类证据；这不是因果判定。',
        '线性斜率仅用于描述慢变趋势，不能排除真实静止条件误差、热漂移或图像变化。',
        'A：角点与 PnP 同时变化时，只能报告共同变化证据。',
        'B：PnP 放大或分支跳变需结合角点、姿态和重投影记录，不能仅凭相关性确认。',
        'C：检查 aimer/preview 的同帧记录、被选择的重建装甲板及其实际匹配的反馈。',
        'D：必须视觉与真实发送均有充分记录才能考虑；未采集真实发送时不能排除此段。','',
        f'时钟身份一致：{clocks_match}；配置变化/读取错误数：{len(config_changes)}；实际发送事件：{len(sends)}。',
        f'采集错误：{capture.get("error")}。','']
    (output/'report.md').write_text('\n'.join(report))
    return summary


def make_plots(output,segments,sends,windows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for w in windows:
        run=w['run']
        start=w['start_ns']/1e9
        fig,axes=plt.subplots(6,1,figsize=(13,16),constrained_layout=True)
        plotted=set()
        for key,label,rows in segments:
            if key[0]!=run:
                continue
            t=[r['host_ns']/1e9-start for r in rows]
            # Colors/labels identify groups; no averaging across identities or frames.
            group=f'{key[1]} id={key[2]} type={key[4]} frame={key[5]}'
            if key[1]=='pnp':
                for i in range(4):
                    for axis in ('u','v'):
                        metric=f'p{i}_{axis}'
                        if all(r.get(metric) is not None for r in rows):
                            tag=(group,metric)
                            axes[0].plot(t,[r[metric] for r in rows],lw=.65,
                                label=f'{group} {metric}' if tag not in plotted else None)
                            plotted.add(tag)
                for metric in ('tx_m','ty_m','tz_m'):
                    if all(r.get(metric) is not None for r in rows):
                        tag=(group,metric)
                        axes[1].plot(t,[r[metric] for r in rows],lw=.7,
                            label=f'{group} {metric}' if tag not in plotted else None)
                        plotted.add(tag)
                for metric in ('normal_heading_camera_rad','normal_elevation_camera_rad'):
                    if all(r.get(metric) is not None for r in rows):
                        tag=(group,metric)
                        axes[2].plot(t,np.rad2deg(np.unwrap([r[metric] for r in rows])),lw=.7,
                            label=f'{group} {metric}' if tag not in plotted else None)
                        plotted.add(tag)
            angle_metrics=('optical_ray_yaw_rad','optical_ray_pitch_rad') if key[1]=='pnp' else ('yaw_rad','pitch_rad')
            for metric in angle_metrics:
                if all(r.get(metric) is not None for r in rows):
                    tag=(group,metric)
                    axes[3].plot(t,np.rad2deg(np.unwrap([r[metric] for r in rows])),lw=.7,
                        label=f'{group} {metric}' if tag not in plotted else None)
                    plotted.add(tag)
        actual=[r for r in sends if r['run']==run and r.get('call_before_ns') is not None]
        if actual:
            t=[r['call_before_ns']/1e9-start for r in actual]
            for metric in ('yaw','pitch'):
                axes[4].plot(t,np.rad2deg(np.unwrap([r[metric] for r in actual])),label=metric)
            axes[5].plot(t[1:],np.diff(t)*1000,'.-',label='host submission intervals')
        else:
            for ax in axes[4:]:
                ax.text(.5,.5,'No actual sends: receive-only capture',ha='center',transform=ax.transAxes)
        for ax,title,ylabel in zip(axes,
            ('Actual PnP image corners (LB,LT,RT,RB)','PnP translation: optical frame',
             'Armor normal orientation in camera frame (offline derived)',
             'Ray / aimer / controller preview: distinct frames, not interchangeable',
             'Actual submitted command angles','Actual send-call intervals'),
            ('pixels','metres','degrees','degrees','degrees','milliseconds')):
            ax.set(title=title,ylabel=ylabel,xlabel='monotonic seconds from run start')
            ax.grid(alpha=.2)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(fontsize=5,loc='upper right',ncol=2)
        fig.savefig(output/f'run_{run}_timeseries.png',dpi=150)
        plt.close(fig)
        # Separate axes expose sub-pixel / millimetre variation hidden by shared offsets.
        detail,axs=plt.subplots(4,3,figsize=(15,12),constrained_layout=True)
        for key,label,rows in segments:
            if key[0]!=run or key[1]!='pnp':
                continue
            t=[r['host_ns']/1e9-start for r in rows]
            for i in range(4):
                for j,axis in enumerate(('u','v')):
                    metric=f'p{i}_{axis}'
                    if all(r.get(metric) is not None for r in rows):
                        axs[i,j].plot(t,[r[metric] for r in rows],lw=.6,color='C0')
                    axs[i,j].set(title=f'PnP corner {i}: {axis}',ylabel='pixels')
            for i,metric in enumerate(('tx_m','ty_m','tz_m','reprojection_mean_px')):
                if all(r.get(metric) is not None for r in rows):
                    scale=1000 if metric.endswith('_m') else 1
                    axs[i,2].plot(t,[r[metric]*scale for r in rows],lw=.6,color='C1')
                axs[i,2].set(title=metric,ylabel='millimetres' if metric.endswith('_m') else 'pixels')
        for ax in axs.flat:
            ax.grid(alpha=.2)
            ax.set_xlabel('monotonic seconds from run start')
        detail.savefig(output/f'run_{run}_corners_position_detail.png',dpi=150)
        plt.close(detail)
        angle_fig,angle_axes=plt.subplots(4,1,figsize=(12,10),constrained_layout=True)
        names=('optical_ray_yaw_rad','optical_ray_pitch_rad',
               'normal_heading_camera_rad','normal_elevation_camera_rad')
        for key,label,rows in segments:
            if key[0]!=run or key[1]!='pnp':
                continue
            t=[r['host_ns']/1e9-start for r in rows]
            for ax,metric in zip(angle_axes,names):
                if all(r.get(metric) is not None for r in rows):
                    ax.plot(t,np.rad2deg(np.unwrap([r[metric] for r in rows])),lw=.6,color='C0')
                ax.set(title=metric,ylabel='degrees',xlabel='monotonic seconds from run start')
                ax.grid(alpha=.2)
        angle_fig.savefig(output/f'run_{run}_angles_detail.png',dpi=150)
        plt.close(angle_fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture',type=Path)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    result=analyze(args.capture,args.output or args.capture/'analysis')
    print(json.dumps({k:result[k] for k in ('runs','association_issues','actual_send_events')},indent=2))


if __name__=='__main__':
    main()
