#!/usr/bin/env python3
"""Print results/summary.csv as a Markdown table: camera estimate and Gazebo truth side by side."""
import csv
import sys


def triple(row, prefix):
    try:
        return '%+.3f / %+.2f / %+.3f' % (float(row[prefix + '_lateral_m']),
                                          float(row[prefix + '_heading_deg']),
                                          float(row[prefix + '_longitudinal_m']))
    except (KeyError, TypeError, ValueError):
        return 'n/a'


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'results/summary.csv'
    print('| run | start, camera | start, Gazebo | aligned, camera | aligned, Gazebo | align time (s) | '
          'camera ok | Gazebo ok | final state | rail end x (m) | rail max lateral (m) | wheels clear |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|')
    with open(path) as handle:
        for row in csv.DictReader(handle):
            print('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
                row['run'], triple(row, 'start_cam'), triple(row, 'start_gt'),
                triple(row, 'end_cam'), triple(row, 'end_gt'), row['duration_s'] or 'n/a',
                row['cam_within_tolerance'], row['gt_within_tolerance'], row['result'],
                row['rail_end_x_m'] or 'n/a', row['rail_max_abs_lateral_m'] or 'n/a',
                row['rail_wheels_clear'] or 'n/a'))


if __name__ == '__main__':
    main()
