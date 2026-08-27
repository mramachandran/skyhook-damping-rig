# Active Quarter-Car Suspension Test Rig

A physical quarter-car suspension rig with a live digital twin, built to test **skyhook active damping** against a passive baseline. Designed and built by Jonathan [Last Name] as a hands-on mechanical & control systems engineering project — from 3D-printed frame and spring selection through embedded control code.

**[Live demo →](https://mramachandran.github.io/skyhook-damping-rig/)**

## What this is

A quarter-car model — one wheel's worth of a car's suspension — instrumented and actuated so it can run in two modes:

- **Passive**: the servo holds a fixed position, springs and damping do the work
- **Active (skyhook)**: an ESP32 reads the plate's motion 100 times a second and commands the servo to counteract it, targeting an *absolute* reference (the "sky") rather than just the relative motion between plate and base

The goal: measure how much skyhook control reduces body displacement versus the passive baseline, under the same disturbance.

## Result

> **26.6% reduction in RMS body displacement** with skyhook active vs. passive, measured over two full logged sessions.
> *(Peak-displacement comparisons are noisier since disturbance timing/magnitude wasn't identical between runs — RMS over the full session is the standard, more robust vibration metric. See `data/README.md` for the raw numbers.)*

**Methodology & honest limitations** (full detail on the live site):
- Position is raw ToF distance, not a calibrated zero — each run is re-centered on its own mean before comparing.
- Peak displacement is actually *higher* on the active run, because that run includes one large deliberate disturbance test the passive run didn't get — not a matched comparison, which is why RMS (not peak) is the headline number.
- The two runs weren't matched in duration or disturbance protocol. A tighter follow-up — same disturbance, same duration, several repeated trials — would make the result more defensible than this single pair of runs.

## How it works

| Component | Role |
|---|---|
| MPU6500 IMU | Acceleration sensing (used as acceleration-only input — integrating for velocity drifts, so it's not used for that) |
| VL53L1X Time-of-Flight sensor | Plate position, differentiated for velocity |
| MG90S servo | Active actuator, ±60° range |
| ESP32-WROOM-32D | 100Hz control loop running the skyhook law, serial telemetry at 20Hz |

Full build log, wiring notes, and debugging history: [`CLAUDE.md`](./CLAUDE.md).

## Repo structure

```
firmware/     ESP32 sketches — integrated control firmware + standalone diagnostics
mechanical/   OpenSCAD source + STL exports for the rig frame/mounts
dashboard/    Live dashboard (Web Serial API) — runs against real hardware, Chrome/Edge only
docs/         GitHub Pages site — project story + replay of a real logged run
data/         Logged CSVs from passive vs. active comparison runs
```

## Running the live dashboard against real hardware

1. Open `dashboard/rig-dashboard.html` in Chrome or Edge (Web Serial API required — no other browser supports it)
2. Plug in the ESP32, click **Connect**, select the serial port
3. Use `SKY:0` / `SKY:1` to toggle passive vs. active control live

## Photos & video

See the [live demo](https://mramachandran.github.io/skyhook-damping-rig/) for build photos and a walkthrough video. Full-res photos are in `docs/media/`.

## Why I built this

[A couple sentences here in your own voice — what got you started, what was hardest, what you'd do differently. This is the part that makes it read as a real project, not a tutorial clone — worth writing yourself.]

## License

MIT — see [LICENSE](./LICENSE)
