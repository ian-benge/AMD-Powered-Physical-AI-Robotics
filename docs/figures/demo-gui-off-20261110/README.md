# Stationary GUI-off measurements

These are actual saved status readings from November 10, 2026, not simulated
temperature values. The 330-second interval contains a 30-second settling period
and five steady minutes. Gray shading marks settling. The figure displays Tctl,
the application's rolling publication p95 and reported real-time factor.

- [PNG](stationary-gui-off.png), [SVG](stationary-gui-off.svg), [PDF](stationary-gui-off.pdf)
- [Observed CSV](readings.csv); no readings interpolated or invented
- [Scope and original failure/handoff](../../AUTONOMOUS_DEMO_BROWSER_PROFILE.md)
- [Selected interval summary](../../../evidence/amd-testing-20261111/thermal/STATIONARY_INTERVAL.json)

Two off-host observers overlapped. The CSV switches streams at 22:14:30 UTC;
the source column identifies the observer. The original single-input failure
and both hashed raw snapshots remain preserved in the private archive. The
complete interval has a maximum 1.014-second status gap.

The rover was stopped. The historical startup odometry-fault latch remained;
current inputs were healthy. The below-80°C goal was not met. This is not a
matched GUI-on comparison, a new aggregate latency benchmark, or autonomous/
endurance acceptance.

With Matplotlib installed, regenerate on CPU from the repository root:

```bash
python scripts/plot_demo_gui_off_thermal.py \
  docs/figures/demo-gui-off-20261110/readings.csv \
  outputs/demo-gui-off-figures
```
