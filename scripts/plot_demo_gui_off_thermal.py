"""Plot the saved stationary GUI-off observations; does not contact Halo."""
import argparse
import csv
from pathlib import Path
from statistics import mean, median

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    with args.csv.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('No observations')
    x = [float(row['elapsed_s']) for row in rows]
    steady = [row for row in rows if 30 <= float(row['elapsed_s']) <= 330]
    if not steady:
        raise ValueError('No steady-period observations')
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.subplots_adjust(top=.85, bottom=.16, left=.10, right=.96, hspace=.22)
    fig.suptitle('Gazebo GUI-off: stationary browser mapping', x=.10, ha='left',
                 fontsize=20, fontweight='bold', y=.96)
    fig.text(.10, .90, 'November 10, 2026 | 30 s warm-up + 300 s observation | Ogre RGB-D at 15 Hz',
             fontsize=11, color='#52647a')
    series = [('tctl_c', 'Tctl (°C)', '#007f78'),
              ('p95_ms', 'Rolling publication\np95 (ms)', '#2764bc'),
              ('rtf', 'Reported real-time\nfactor', '#7350a1')]
    for ax, (key, label, color) in zip(axes, series):
        ax.plot(x, [float(row[key]) for row in rows], color=color, linewidth=1.5)
        ax.axvspan(0, 30, color='#e7edf3', alpha=.7)
        ax.set_ylabel(label, fontsize=10)
        ax.grid(axis='y', color='#dde5ed', linewidth=.8)
        ax.spines[['top', 'right']].set_visible(False)
        ax.set_xlim(0, 330)
    axes[0].axhline(80, color='#b16b08', linestyle='--', linewidth=1, label='80°C target')
    axes[0].axhline(87, color='#bd4144', linestyle='--', linewidth=1, label='87°C qualification stop')
    axes[0].set_ylim(76, 89)
    axes[0].legend(loc='upper left', frameon=False, fontsize=9, ncol=2)
    axes[1].axhline(200, color='#bd4144', linestyle='--', linewidth=1)
    axes[1].set_ylim(140, 208)
    axes[2].axhline(.8, color='#bd4144', linestyle='--', linewidth=1)
    axes[2].set_ylim(.76, 1.03)
    axes[2].set_xlabel('Seconds since observation began (wall clock)')
    fig.text(.10, .075, f"Steady readings: mean {mean(float(r['tctl_c']) for r in steady):.2f}°C, "
             f"peak {max(float(r['tctl_c']) for r in steady):.2f}°C; worst rolling p95 "
             f"{max(float(r['p95_ms']) for r in steady):.2f} ms; median RTF "
             f"{median(float(r['rtf']) for r in steady):.3f}.",
             fontsize=10, color='#172b46')
    fig.text(.10, .045, 'Below-80°C target not met. No matched baseline; no autonomous or endurance acceptance.',
             fontsize=10, color='#52647a')
    fig.text(.10, .020, 'Rover stayed stopped. Historical startup odometry fault retained; current inputs were healthy.',
             fontsize=9, color='#52647a')
    args.output.mkdir(parents=True, exist_ok=True)
    for suffix in ('png', 'svg', 'pdf'):
        fig.savefig(args.output / ('stationary-gui-off.' + suffix), dpi=180, facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    main()
