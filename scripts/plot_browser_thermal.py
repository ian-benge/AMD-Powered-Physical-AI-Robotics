"""Render the frozen November 8 thermal evidence with CPU-only Matplotlib.

Run: python scripts/plot_browser_thermal.py
The input contains saved observations, not interpolated one-second readings.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'docs' / 'figures' / 'browser-thermal-20261108'
INK = '#172b46'
MUTED = '#52647a'
GRID = '#dde5ed'
TEAL = '#007f78'
BLUE = '#2764bc'
RED = '#bd4144'
AMBER = '#b16b08'


def frame(title, subtitle):
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    fig.text(.055, .935, title, fontsize=26, fontweight='bold', color=INK)
    fig.text(.055, .889, subtitle, fontsize=15, color=MUTED)
    return fig


def style(ax):
    ax.spines[['top', 'right']].set_visible(False)
    ax.spines[['left', 'bottom']].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=13, length=0, pad=9)
    ax.set_axisbelow(True)
    ax.grid(axis='y', color=GRID, linewidth=.8)
    ax.xaxis.label.set_color(MUTED)
    ax.yaxis.label.set_color(MUTED)


def thermal_axis(ax, limit):
    style(ax)
    ax.set_ylim(78, 92)
    ax.set_yticks([80, 82, 84, 86, 88, 90])
    ax.set_ylabel('Tctl temperature (°C)', fontsize=14)
    ax.set_xlabel('Seconds since observation began', fontsize=14, labelpad=10)
    ax.axhline(limit, color=RED, linestyle=(0, (5, 4)), linewidth=1.6)
    ax.text(.985, .90, '90°C native stop', transform=ax.transAxes,
            ha='right', fontsize=12, color=RED,
            bbox=dict(facecolor='white', edgecolor='none', pad=2))


def save(fig, out, name):
    for extension in ('png', 'svg', 'pdf'):
        fig.savefig(out / f'{name}.{extension}', dpi=180, facecolor='white')
    plt.close(fig)


def successful_checks(data, out):
    runs = data['runs']
    fig = frame('15 Hz browser assessment: temperature checks',
                'Measured AMD Tctl • November 8, 2026 (America/Chicago) • new epoch-22 NPU detector')
    axes = [fig.add_axes([.075, .25, .40, .55]),
            fig.add_axes([.565, .25, .40, .55])]
    for ax, ident, color, title in zip(axes, ('stationary_15hz', 'drive_15hz'),
                                     (TEAL, BLUE), ('Stationary mapping · 5 minutes',
                                                    'Driving check · 2-minute observation')):
        run = runs[ident]
        rows = run['samples']
        thermal_axis(ax, data['native_stop_c'])
        ax.set_title(title, fontsize=18, color=INK, loc='left', pad=18)
        ax.plot([r['seconds'] for r in rows], [r['tctl_c'] for r in rows],
                color=color, linewidth=2.5, marker='o', markersize=4,
                label='Saved temperature readings')
        ax.set_xlim(0, 305 if ident == 'stationary_15hz' else 123)
        if ident == 'drive_15hz':
            moving = [r for r in rows if any(r['command'] or [])]
            ax.scatter([r['seconds'] for r in moving], [r['tctl_c'] for r in moving],
                       s=85, facecolors='white', edgecolors=AMBER, linewidths=2,
                       zorder=5, label='Nonzero command samples')
            ax.legend(loc='lower left', frameon=False, fontsize=12)
        else:
            ax.set_xticks([0, 60, 120, 180, 240, 300])
        peak = run['result']['max_tctl_c']
        ax.text(.035, .775, f'Recorded peak: {peak:g}°C', transform=ax.transAxes,
                fontsize=17, fontweight='bold', color=color)
    fig.text(.075, .135, '31 / 31 camera and localization samples healthy',
             color=TEAL, fontsize=16, fontweight='bold')
    fig.text(.565, .135, '25 / 25 camera and localization samples healthy',
             color=BLUE, fontsize=16, fontweight='bold')
    fig.text(.055, .075,
             'Left: readings about every 10 s. Right: saved status snapshots about every 5 s; '
             '86.125°C peak comes from the 1 s monitor receipt.', fontsize=12, color=MUTED)
    fig.text(.055, .041,
             'Motion was observed in six snapshots near the end of the right-hand window. '
             'These bounded checks do not establish endurance stability.', fontsize=12, color=MUTED)
    save(fig, out, '01_temperature_checks')


def iterations(data, out):
    runs = data['runs']
    fig = frame('Thermal repair: measured iteration results',
                'Removing RViz alone and limiting CPU threads still led to early thermal verification stops.')
    ax = fig.add_axes([.075, .26, .42, .53])
    thermal_axis(ax, data['native_stop_c'])
    ax.set_title('Saved temperature readings', fontsize=18, loc='left', color=INK, pad=18)
    definitions = [('rviz_off', RED, '30 Hz · RViz off · 80 s'),
                   ('thread_caps', AMBER, '30 Hz · + CPU/thread caps · 10 s'),
                   ('stationary_15hz', TEAL, '15 Hz · + lower live cadence · 301 s')]
    for ident, color, label in definitions:
        rows = runs[ident]['samples']
        ax.plot([r['seconds'] for r in rows], [r['tctl_c'] for r in rows],
                color=color, marker='o', markersize=4, linewidth=2.2, label=label)
        if runs[ident]['result']['stopped_early']:
            ax.scatter(rows[-1]['seconds'], rows[-1]['tctl_c'], marker='X',
                       s=110, color=color, zorder=5)
    ax.set_xlim(0, 305)
    ax.set_xticks([0, 60, 120, 180, 240, 300])
    ax.legend(loc='lower right', fontsize=11, frameon=False)
    power = fig.add_axes([.71, .28, .245, .49])
    style(power)
    power.grid(False)
    values = [runs[k]['result']['mean_ppt_w'] for k, _, _ in definitions]
    power.barh(range(3), values, color=[d[1] for d in definitions], height=.5)
    power.set_yticks(range(3), ['RViz off\n9 samples / 80 s',
                              '+ Thread caps\n2 samples / 10 s',
                              '+ 15 Hz camera\n31 samples / 301 s'])
    power.invert_yaxis()
    power.set_xlim(0, 104)
    power.set_xticks([0, 25, 50, 75, 100])
    power.set_xlabel('Mean package PPT (W)', fontsize=14, labelpad=12)
    power.set_title('Observed package power', fontsize=18, loc='left', color=INK, pad=30)
    for i, value in enumerate(values):
        power.text(value + 1.5, i, f'{value:.2f}', va='center', fontsize=15,
                   fontweight='bold', color=INK)
    fig.text(.055, .145, 'Final stationary interval: 70.60 W mean package power; 84.75°C peak.',
             fontsize=18, fontweight='bold', color=TEAL)
    fig.text(.055, .085,
             'Separate runs with different starting temperatures, motion and durations. '
             'The 10 s thread-cap interval contains only two samples.', fontsize=12, color=MUTED)
    fig.text(.055, .05,
             'Early observers stopped at ≥85°C on sampled checks; native protection stayed at 90°C. '
             'These are observed intervals, not a controlled performance benchmark.', fontsize=12, color=MUTED)
    save(fig, out, '02_repair_iterations')


def changes(out):
    fig = frame('What changed in the browser assessment',
                'Reduce duplicate work, bound CPU parallelism, lower live sensor cadence, and preserve explicit failures.')
    cards = [(.055, .54, '1', 'Remove the duplicate display',
              'RViz disabled for the browser assessment.\nGazebo minimized after launch inspection.\nQualified Ogre sensor rendering retained.'),
             (.525, .54, '2', 'Bound CPU and thread use',
              'Container restricted to 8 physical cores.\nOpenMP: 2 workers, thread limit 4.\nBLAS: 2 workers; passive OpenMP waiting.'),
             (.055, .25, '3', 'Reduce live RGB-D to 15 Hz',
              'Live cadence reduced from 30 to 15 Hz.\n960 × 540 geometry and clipping retained.\nCapture and legacy defaults remain 30 Hz.'),
             (.525, .25, '4', 'Report the real stop cause',
              'Read terrain stop evidence before PID lookup.\nRetry only a vanished login-session race.\nDo not retry completed terrain attempts.')]
    for x, y, number, title, body in cards:
        rect = FancyBboxPatch((x, y), .42, .24,
                              boxstyle='round,pad=0.014,rounding_size=0.012',
                              transform=fig.transFigure, facecolor='#f2f6fa',
                              edgecolor=GRID, linewidth=1)
        fig.patches.append(rect)
        fig.text(x + .014, y + .179, number, fontsize=20,
                 color=TEAL, fontweight='bold')
        fig.text(x + .048, y + .179, title, fontsize=19, color=INK, fontweight='bold')
        fig.text(x + .015, y + .064, body, fontsize=16, linespacing=1.65, color=MUTED)
    fig.text(.055, .145, 'Retained: 90°C stop · sensor freshness · motion guards · float64 terrain · HIP flags 4',
             fontsize=17, color=TEAL, fontweight='bold')
    fig.text(.055, .09, 'Verified: 16 Linux regression tests, five minutes of stationary mapping, and a brief 0.05 m/s drive.',
             fontsize=15, color=INK)
    fig.text(.055, .05, 'The new model and accepted launchers were preserved. '
             'The successful checks cover this browser profile; longer endurance remains unqualified.',
             fontsize=12, color=MUTED)
    save(fig, out, '03_what_changed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements', type=Path, default=DEFAULT / 'measurements.json')
    parser.add_argument('--output', type=Path, default=DEFAULT)
    args = parser.parse_args()
    data = json.loads(args.measurements.read_text(encoding='utf-8-sig'))
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'svg.fonttype': 'none',
                         'pdf.fonttype': 42, 'axes.titleweight': 'bold'})
    successful_checks(data, out)
    iterations(data, out)
    changes(out)
    with (out / 'thermal_readings.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['run', 'utc', 'elapsed_seconds', 'tctl_c', 'package_ppt_w',
                         'linear_command_mps', 'angular_command_radps'])
        for name, run in data['runs'].items():
            for row in run['samples']:
                writer.writerow([name, row['utc'], row['seconds'], row['tctl_c'], row['ppt_w'],
                                 *(row['command'] or [None, None])])
    manifest = {'measurements_sha256': hashlib.sha256(args.measurements.read_bytes()).hexdigest(),
                'matplotlib_version': matplotlib.__version__,
                'files': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(out.iterdir()) if p.suffix in ('.png', '.svg', '.pdf', '.csv')}}
    (out / 'GRAPH_MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'figures': 3, 'formats': ['png', 'svg', 'pdf'],
                      'saved_temperature_readings': sum(len(r['samples']) for r in data['runs'].values())}))


if __name__ == '__main__':
    main()
