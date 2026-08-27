# Comparison run data

- `rig_run_1_skyon.csv` — skyhook active (SKY:1)
- `rig_run_2_skyoff.csv` — passive baseline (SKY:0)

Columns: `time_s, pos_mm, vel_mms, accel_g, u_cmd_N, servo_deg, sky_on`

`pos_mm` is raw ToF distance, not a calibrated zero — the site re-centers each run on its own mean when computing displacement/RMS, since the two runs settle at slightly different rest distances.

Result: **26.6% RMS displacement reduction** with skyhook active vs. passive (RMS 3.75mm vs 5.10mm). See `docs/index.html` section 01 for the full table and replay chart.
