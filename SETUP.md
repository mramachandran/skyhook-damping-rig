# Getting this live

## 1. Copy in your real files
- `firmware/`: `rig_control_v3.ino`, `wiring_diagnostic.ino`, `mpu_tilt_test.ino`, `servo_test.ino`
- `mechanical/`: OpenSCAD source + any exported STLs
- `dashboard/`: `rig-dashboard.html`
- `CLAUDE.md`: your existing project log, copy to repo root
- `data/comparison_run.csv`: once you log the SKY:0 vs SKY:1 comparison run, export it with columns `t,passive_mm,active_mm` (or similar — adjust the loader in `docs/index.html` if your column names differ). Until this file exists, the site shows clearly-labeled placeholder data.

## 2. Photos
Drop JPGs into `docs/media/` and update the `<figure>` tags in `docs/index.html` to point to them (there are commented examples in the file).

## 3. Video
1. Upload your walkthrough video to YouTube (unlisted is fine — still linkable, just not searchable)
2. Copy the video ID from the URL (`youtube.com/watch?v=`**`THIS_PART`**)
3. Paste it into the `src="https://www.youtube.com/embed/YOUR_VIDEO_ID"` line in `docs/index.html`

## 4. Push to GitHub
```bash
cd skyhook-damping-rig
git init
git add .
git commit -m "Initial commit: quarter-car active suspension rig"
git branch -M main
git remote add origin https://github.com/mramachandran/skyhook-damping-rig.git
git push -u origin main
```

## 5. Turn on GitHub Pages
1. On the repo page: **Settings → Pages**
2. Under "Build and deployment", set **Source: Deploy from a branch**
3. Branch: `main`, folder: `/docs`
4. Save — your site will be live at `https://mramachandran.github.io/skyhook-damping-rig/` within a minute or two

## 6. Fill in the placeholders
Search the repo for `mramachandran` and `XX%` and `YOUR_VIDEO_ID` and replace them. The result stat on the landing page auto-computes from `data/comparison_run.csv` once it's real, but the README's number is written by hand — update it too.
