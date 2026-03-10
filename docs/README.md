# GitHub Pages Interactive Trajectory Viewer

This folder hosts a static interactive 3D viewer for OMPL benchmark trajectories.

## 1) Export data into `docs/data/latest`

From repo root:

```bash
python3 ompl_cpp_bench/4roomversion/export_github_pages_data.py \
  --results_dir ompl_cpp_bench/4roomversion/results/fixed_v2_first_solution_20260308 \
  --docs_dir docs \
  --target latest
```

This creates:
- `docs/data/latest/manifest.json`
- CSV files needed by the viewer.

## 2) Preview locally (optional)

From repo root:

```bash
python3 -m http.server 8000
```

Then open:
- `http://localhost:8000/docs/`

## 3) Publish on GitHub Pages

On GitHub:
1. Go to `Settings` -> `Pages`
2. Set `Source` to `Deploy from a branch`
3. Branch: `main` (or your presentation branch), folder: `/docs`
4. Save and wait for deployment

Your page URL will look like:
- `https://<your-username>.github.io/<repo-name>/`

