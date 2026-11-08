#!/usr/bin/env python3
"""Compare recorded velocities with explicitly labelled offline model traces."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def recorded(path):
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.array([float(row[key]) for row in rows])
            for key in ('t', 'vx', 'vy', 'wz')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis_dir', type=Path)
    args = parser.parse_args()
    root = args.analysis_dir
    cases = json.loads((root / 'motion_final_results.json').read_text())
    selected = [next(case for case in cases
                     if case['case'].startswith(prefix) and case['response_lag'] == .3)
                for prefix in ('recorded_outbound', 'recorded_return')]
    selected.append(max((case for case in cases if case['case'].startswith('requested')),
                        key=lambda case: (case['linear_zero_after_walk'], case['duration'])))
    fig, axes = plt.subplots(4, 2, figsize=(14, 12), constrained_layout=True)
    command_color, feedback_color = '#087e8b', '#d88126'
    command = recorded(root / 'dog_safe_cmd_vel.csv')
    feedback = recorded(root / 'dog_odom.csv')

    def curves(row, command, feedback, title, model=False):
        planar = axes[row, 0]
        angular = axes[row, 1]
        planar.plot(command['t'], np.hypot(command['vx'], command['vy']),
                    color=command_color, label='Command', lw=1.5)
        planar.plot(feedback['t'], np.hypot(feedback['vx'], feedback['vy']),
                    color=feedback_color, label='Synthetic feedback' if model else 'Robot feedback',
                    lw=1., alpha=.8)
        angular.plot(command['t'], command['wz'], color=command_color, lw=1.5)
        angular.plot(feedback['t'], feedback['wz'], color=feedback_color, lw=1., alpha=.8)
        planar.set(title=title, ylabel='Planar speed (m/s)', ylim=(-.025, .7))
        angular.set(title='Body yaw rate', ylabel='rad/s', ylim=(-.65, .65))
        planar.legend(loc='upper right', fontsize=8)
        for ax in (planar, angular):
            ax.grid(alpha=.2)
            ax.set_xlabel('Model elapsed time (s)' if model else 'Bag elapsed time (s)')
        if model:
            planar.axhline(.21, color='gray', ls=':', lw=.7)

    curves(0, command, feedback, 'Recorded bag: actual command and robot feedback')
    for row, case in enumerate(selected, start=1):
        samples = case['samples']
        command = {key: np.array([sample[key] for sample in samples])
                   for key in ('t', 'vx', 'vy', 'wz')}
        feedback = {'t': command['t']}
        for key in ('vx', 'vy', 'wz'):
            feedback[key] = np.array([sample['feedback_' + key] for sample in samples])
        name = ('Outbound' if row == 1 else 'Return' if row == 2 else 'Requested far goal')
        title = (f'Offline model: {name}; lag {case["response_lag"]:.2f} s; '
                 f'in-route zero {case["linear_zero_after_walk"]:.2f} s')
        curves(row, command, feedback, title, model=True)
        first_walk = case['first_walk']
        settling = next((sample['t'] for sample in samples
                         if sample['phase'] == 'settling' and sample['t'] >= first_walk),
                        samples[-1]['t'])
        for ax in axes[row]:
            ax.axvspan(0, first_walk, color='gray', alpha=.12)
            ax.axvspan(settling, samples[-1]['t'], color='gray', alpha=.12)
            for sample in samples:
                if (sample['t'] >= first_walk and sample['phase'] not in ('settling', 'goal_align')
                        and np.hypot(sample['vx'], sample['vy']) < 1e-9):
                    ax.axvspan(sample['t'], sample['t'] + .05, color='#b72f3c', alpha=.18)
    fig.suptitle('Saved-map offline motion audit — model results are not robot measurements\n'
                 'Gray: initial alignment / final settling excluded from in-route zero time', fontsize=13)
    output = root / 'motion_final_velocity.png'
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
