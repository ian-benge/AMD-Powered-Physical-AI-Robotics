# Temperature graphs and assessment changes

Presentation figures from the November 8, 2026 browser assessment (America/Chicago).
The source timestamps fall on November 9 in UTC. These measure **AMD Tctl**, not
a separate GPU temperature sensor.

| Figure | Use |
| --- | --- |
| [Temperature checks](01_temperature_checks.png) | Five-minute stationary mapping and the separate driving observation, with the retained 90°C stop shown. |
| [Repair iterations](02_repair_iterations.png) | Earlier thermal stops, final temperature readings, and observed mean package power. |
| [What changed](03_what_changed.png) | Removed RViz, bounded CPU/thread use, selected 15 Hz live RGB-D, and improved stop reporting. |

Each figure is available as a 2880 × 1620 PNG, an editable SVG and a vector PDF
with the same filename stem. PNGs fit a 16:9 presentation slide.

Suggested caption: **“The 15 Hz browser profile completed five minutes of active
mapping and a brief 0.05 m/s driving check. Recorded Tctl peaks were 84.75°C and
86.125°C, below the unchanged 90°C stop.”**

The final profile retained 960 × 540 camera geometry, qualified Ogre rendering,
float64 terrain processing, HIP flags 4, freshness rules and motion safeguards.
Image-collection and legacy camera defaults remain 30 Hz.

## Reading the measurements

- The stationary plot contains 31 readings about 10 seconds apart.
- The driving plot contains 25 saved status snapshots about 5 seconds apart.
  The separate one-second monitoring receipt reports a peak of 86.125°C; the
  highest saved five-second snapshot is 85.875°C. The missing one-second series
  has not been reconstructed or plotted.
- Driving appears in six nonzero-command snapshots near the end of the
  120-second observation window. This is not 120 seconds of continuous driving.
- The comparison uses separate, unequal observation intervals: RViz off
  (9 samples / 80 seconds), added thread limits (2 samples / 10 seconds), and
  the final stationary profile (31 samples / 301 seconds). Starting temperatures
  and motion differed. The power bars describe these observations and do not
  isolate the causal effect of any one change.
- Early verification helpers stopped their identified assessment at a sampled
  temperature of at least 85°C. The native hard limit stayed at 90°C. The
  original 90°C trip is documented in the repair report; its time series is not
  included in these plots.
- These short checks do not establish endurance stability or higher-speed
  acceptance for this profile.

## Data and reproduction

[measurements.json](measurements.json) freezes all 67 saved temperature readings,
result receipts and hashes of the original JSON/JSONL evidence. Process listings,
host identities and model paths are excluded. [thermal_readings.csv](thermal_readings.csv)
is the corresponding table. `GRAPH_MANIFEST.json` records SHA-256 hashes of the
input and rendered files.

From the repository root, with Matplotlib installed:

```bash
python scripts/plot_browser_thermal.py
```

Rendering uses the CPU-only Agg backend. No live simulation, training or GPU
measurement is started. See the [full repair report](../../CAPTURE5000_BROWSER_THERMAL_FIX.md)
for the retained stop evidence and implementation history.
