#!/usr/bin/env python3
"""Offline, read-only SQLite ROS 2 bag analysis; never publishes ROS messages."""
import argparse
import csv
import json
import math
import sqlite3
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def xyz(p):
    return [p.x, p.y, p.z]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--ground-offset', type=float)
    parser.add_argument('--tracking-limit', type=float)
    args = parser.parse_args()
    session = args.session.resolve()
    out = session/'analysis'
    out.mkdir(exist_ok=True)
    parameters = {}
    parameter_source = 'explicit_overrides'
    if args.ground_offset is None or args.tracking_limit is None:
        try:
            parameters = yaml.safe_load((session/'context/parameters/navigation_executor.yaml').read_text())
            parameters = parameters['/navigation_executor']['ros__parameters']
            parameter_source = 'runtime_parameter_dump'
        except (OSError, yaml.YAMLError, KeyError, TypeError):
            robot = json.loads((session/'context/robot.json').read_text())
            config = yaml.safe_load((session/'context/config/src/dog_navigation/config/local_planner.yaml').read_text())
            parameters = dict(base_ground_offset=robot['navigation']['base_ground_offset'],
                              max_tracking_error=config['navigation_executor']['ros__parameters']['max_tracking_error'])
            parameter_source = 'runtime_robot_ground_offset_and_saved_config_tracking_limit'
            print('参数 dump 不可读：接地偏移使用运行快照，跟踪容差使用配置快照。', file=sys.stderr)
    ground_offset = float(args.ground_offset if args.ground_offset is not None else parameters['base_ground_offset'])
    tracking_limit = float(args.tracking_limit if args.tracking_limit is not None else parameters['max_tracking_error'])
    if not all(math.isfinite(v) and v > 0 for v in (ground_offset, tracking_limit)):
        parser.error('ground offset and tracking limit must be finite and positive')
    records = []
    for db in sorted((session/'bag').glob('*.db3')):
        with sqlite3.connect(db.as_uri()+'?mode=ro', uri=True) as con:
            topics = dict((i, (name, kind)) for i, name, kind in con.execute('select id,name,type from topics'))
            selected = {i: (name, get_message(kind)) for i, (name, kind) in topics.items()
                        if kind != 'sensor_msgs/msg/PointCloud2' and name != '/interfaces/imu'}
            query = 'select timestamp,topic_id,data from messages where topic_id in ('+','.join('?' for _ in selected)+') order by timestamp,id'
            for stamp, topic_id, data in con.execute(query, tuple(selected)):
                name, kind = selected[topic_id]
                records.append((stamp, name, deserialize_message(data, kind)))
    records.sort(key=lambda x: x[0])
    start = records[0][0]
    streams = defaultdict(list)
    tf = {}
    for stamp, name, msg in records:
        t = (stamp-start)*1e-9
        if name == '/tf' or name == '/tf_static':
            for item in msg.transforms:
                v = item.transform
                tf[(item.header.frame_id, item.child_frame_id)] = (rotation(v.rotation), np.array(xyz(v.translation)))
            if ('map', 'odom') in tf and ('odom', 'base_link') in tf:
                r, p = tf['map', 'odom']; rb, pb = tf['odom', 'base_link']
                ground = r@(pb+rb@np.array([0, 0, -ground_offset]))+p
                streams['ground'].append(dict(t=t, xyz=ground.tolist(),
                    yaw=math.atan2((r@rb)[1, 0], (r@rb)[0, 0]),
                    odom_xyz=pb.tolist(), correction_xyz=p.tolist(),
                    correction_yaw=math.atan2(r[1, 0], r[0, 0])))
        elif name.endswith('/status') and hasattr(msg, 'data'):
            try:
                streams[name].append(dict(t=t, **json.loads(msg.data)))
            except (ValueError, TypeError):
                pass
        elif name in ('/navigation/command', '/dog/safe_cmd_vel', '/dog/odom', '/lio/odom', '/cmd_vel'):
            v = msg.velocity if name == '/navigation/command' else msg.twist.twist if name.endswith('/odom') else msg
            row = dict(t=t, vx=v.linear.x, vy=v.linear.y, vz=v.linear.z,
                       wx=v.angular.x, wy=v.angular.y, wz=v.angular.z)
            if name == '/navigation/command':
                row.update(phase=msg.phase, permit=msg.permit, generation=msg.route_generation, task_id=msg.task_id)
            if name.endswith('/odom'):
                row['xyz'] = xyz(msg.pose.pose.position)
            streams[name].append(row)
        elif name == '/local_navigation/terrain_route':
            streams[name].append(dict(t=t, generation=msg.generation, valid=msg.valid,
                reason=msg.reason, points=[xyz(p) for p in msg.points], stairs=list(msg.stairs),
                goal=xyz(msg.goal), goal_stamp=[msg.goal_stamp.sec, msg.goal_stamp.nanosec]))
        elif name == '/rosout':
            streams[name].append(dict(t=t, node=msg.name, level=msg.level, message=msg.msg))
        elif name == '/navigation/planner_goal':
            streams[name].append(dict(t=t, xyz=xyz(msg.pose.position)))
    for name in ('/navigation/command', '/dog/safe_cmd_vel', '/dog/odom', '/lio/odom'):
        rows = streams[name]
        if not rows:
            continue
        fields = list(rows[0])
        with (out/(name.strip('/').replace('/', '_')+'.csv')).open('w') as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)
    (out/'timeline.json').write_text(json.dumps(streams, ensure_ascii=False))
    summary = dict(duration=(records[-1][0]-start)*1e-9, start_unix_ns=start,
                   parameter_source=parameter_source,
                   base_ground_offset=ground_offset, max_tracking_error_limit=tracking_limit,
                   velocities={}, tasks={}, guard_counts=dict(Counter(r['reason'] for r in streams['/velocity_guard/status'])))
    for name in ('/navigation/command', '/dog/safe_cmd_vel', '/dog/odom', '/lio/odom'):
        rows = streams[name]
        summary['velocities'][name] = {key: [min(r[key] for r in rows), max(r[key] for r in rows)]
                                       for key in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz')} if rows else {}
    for row in streams['/navigation_executor/status']:
        task = summary['tasks'].setdefault(row['task_id'], dict(first=row['t'], phases=Counter(), max_tracking_error=0, transitions=[]))
        task['last'] = row['t']; task['phases'][row['phase']] += 1
        task['max_tracking_error'] = max(task['max_tracking_error'], row['tracking_error'])
        key = (row['task_state'], row['phase'], row['reason'])
        if not task['transitions'] or tuple(task['transitions'][-1][1:]) != key:
            task['transitions'].append([row['t'], *key])
    states = streams['/navigation_executor/status']
    phases = Counter()
    for a, b in zip(states, states[1:]):
        if a['task_state'] not in ('cancelled', 'idle', 'aborted', 'succeeded'):
            phases[a['phase']] += b['t']-a['t']
    summary['active_phase_seconds'] = dict(phases)
    summary['planner_states'] = dict(Counter(r['state'] for r in streams['/local_navigation/status']))
    summary['planner_max_processing_ms'] = max(r['last_processing_ms'] for r in streams['/local_navigation/status'])
    summary['localization_states'] = dict(Counter(r['state'] for r in streams['/map_localizer/status']))
    feedback = streams['/dog/odom']
    feedback_times = np.array([r['t'] for r in feedback])
    zero_align = [r for r in streams['/navigation/command']
                  if r['phase']=='align' and abs(r['vx'])+abs(r['wz'])<1e-8]
    matched = []
    for row in zero_align:
        index = np.searchsorted(feedback_times, row['t'], side='right')-1
        if index >= 0:
            sample = feedback[index]
            matched.append(dict(t=row['t'], feedback_age=row['t']-sample['t'],
                                planar_speed=math.hypot(sample['vx'], sample['vy'])))
    summary['zero_align_commands'] = len(zero_align)
    summary['zero_align_with_feedback_above_stop_threshold'] = sum(r['planar_speed']>.025 for r in matched)
    summary['zero_align_feedback_max_age_seconds'] = max((r['feedback_age'] for r in matched), default=None)
    (out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))

    colors = ['#2674b8', '#dd741e', '#28914d']
    fig, axes = plt.subplots(6, 1, figsize=(15, 15), sharex=True)
    for name, label, color in zip(('/navigation/command', '/dog/safe_cmd_vel', '/dog/odom'),
                                  ('Executor command', 'Guard output', 'Chassis feedback'), colors):
        rows = streams[name]
        for ax, key in zip(axes[:3], ('vx', 'vy', 'wz')):
            ax.step([r['t'] for r in rows], [r[key] for r in rows], where='post', label=label, color=color, linewidth=1, alpha=.8)
    for ax, title in zip(axes[:3], ('Forward vx (m/s)', 'Lateral vy (m/s)', 'Yaw rate (rad/s)')):
        ax.set_ylabel(title); ax.grid(alpha=.25); ax.legend(loc='upper right', ncol=3)
    rows = streams['/navigation_executor/status']
    axes[3].plot([r['t'] for r in rows], [r['tracking_error'] for r in rows], color='#944b99')
    axes[3].axhline(tracking_limit, color='red', ls='--', label=f'Configured XY tracking limit = {tracking_limit:g} m')
    axes[3].set_ylabel('Tracking error (m)'); axes[3].grid(alpha=.25); axes[3].legend()
    reasons = list(dict.fromkeys(r['reason'] for r in streams['/velocity_guard/status']))
    rows = streams['/velocity_guard/status']
    axes[4].step([r['t'] for r in rows], [reasons.index(r['reason']) for r in rows], where='post', color='#52606d')
    axes[4].set_yticks(range(len(reasons)), reasons, fontsize=8)
    axes[4].grid(alpha=.25)
    phase_names = list(dict.fromkeys(r['phase'] for r in states))
    axes[5].step([r['t'] for r in states], [phase_names.index(r['phase']) for r in states], where='post', color='#944b99')
    axes[5].set_yticks(range(len(phase_names)), phase_names, fontsize=8)
    axes[5].grid(alpha=.25); axes[5].set_xlabel('Seconds since first recorded message (bag receive time)')
    for task in summary['tasks'].values():
        for t, state, phase, reason in task['transitions']:
            if state == 'aborted':
                for ax in axes:
                    ax.axvline(t, color='red', ls=':', alpha=.65)
                axes[0].text(t, .98, f' abort {t:.2f}s', transform=axes[0].get_xaxis_transform(), rotation=90, va='top', color='red')
    fig.suptitle('Recorded navigation: requested / permitted / measured velocity\nNo interpolation or low-pass filtering; chassis feedback is not ground truth')
    fig.tight_layout(); fig.savefig(out/'velocity.png', dpi=150)
    # Enlarged interval where repeated waypoint stops dominate forward motion.
    axes[0].set_xlim(36, 48)
    fig.savefig(out/'velocity_detail.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 9))
    unique = set()
    for route in streams['/local_navigation/terrain_route']:
        key = tuple(tuple(p) for p in route['points'])
        if route['valid'] and len(key)>1 and key not in unique:
            unique.add(key); p = np.array(key)
            ax.plot(p[:, 0], p[:, 1], color='#b5bbc4', alpha=.55, lw=1)
    ground = streams['ground']
    if ground:
        p = np.array([r['xyz'] for r in ground])
        dots = ax.scatter(p[:, 0], p[:, 1], c=[r['t'] for r in ground], s=5, cmap='viridis')
        fig.colorbar(dots, ax=ax, label='Time (s)')
    for row in streams['/navigation/planner_goal']:
        ax.scatter(*row['xyz'][:2], marker='*', color='red', s=90)
    ax.set(xlabel='Map X (m)', ylabel='Map Y (m)', title='Recorded terrain routes (grey), estimated ground position, goals (red)')
    ax.set_aspect('equal'); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(out/'trajectory.png', dpi=150); plt.close(fig)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print('Artifacts:', out)


if __name__ == '__main__':
    main()
