# triwild sweep scripts

Batch-run triwild over a large 2D dataset, collect per-model results, and report — plus a
viewer for looking at one result once the numbers point at it.
Written for the 20k 2D curve dataset on `kirby.cs.nyu.edu`, but nothing here is
kirby-specific beyond the default paths.

| file | what it is |
|---|---|
| `run_triwild_sweep.py` | the driver: one triwild run per model, capped in time and memory, then the report |
| `sweep2d.sh` | tmux wrapper — start / stop / attach / status / report |
| `failure_report.py` | post-hoc: where each failed model died, how far it got, at what energy |
| `visualize_triwild.py` | polyscope viewer for a single result: input curves, envelope curves, output mesh |

## Layout

Everything hangs off one root, `TRIWILD_ROOT` (default `/u/3/daniele/triwild-sweep`):

```
<root>/build/app/wmtk_app     the binary under test
<root>/data/*.obj             the dataset
<root>/runs/<name>/           output: success/  failure/  .work/  report/
```

A model that exits 0 lands in `success/<id>/`, anything else in `failure/<id>/` with a
`status.txt` recording the reason. Both keep the config, the log and the report.json.

## Running

```sh
export TRIWILD_ROOT=/path/to/sweep
./sweep2d.sh full          # every model, filename order
./sweep2d.sh quick         # 300 models spread across the size range
./sweep2d.sh status        # progress for every run
./sweep2d.sh stop          # clean stop: drain, write the report, exit
./sweep2d.sh attach        # attach to the tmux session
./sweep2d.sh report [full] # regenerate the report without running anything
```

**Resuming is just re-running the same subcommand** — models already in `success/` or
`failure/` are skipped. A clean `stop` abandons in-flight models *unpublished*, so they
are reprocessed on resume rather than being recorded as spurious failures. To retry
failures under new settings, move `failure/` aside first; anything left there is treated
as already done.

Or drive the runner directly:

```sh
TRIWILD_ROOT=... TRIWILD_OUT=.../runs/full TRIWILD_PARALLEL=13 TRIWILD_THREADS=8 \
TRIWILD_JOB_TIMEOUT=21600 python3 run_triwild_sweep.py
```

### Environment

| var | default | notes |
|---|---|---|
| `TRIWILD_ROOT` | the kirby path | `build/`, `data/`, `runs/` live here |
| `TRIWILD_OUT` | `<root>/runs/full` | output directory |
| `TRIWILD_PARALLEL` | 16 | models at once |
| `TRIWILD_THREADS` | 8 | threads per model |
| `TRIWILD_JOB_TIMEOUT` | 3600 | seconds per model |
| `TRIWILD_MEM_GB` | 128 | per model, 0 disables |
| `TRIWILD_LIMIT` | 0 | process at most N new models |
| `TRIWILD_SAMPLE` | `name` | `name` \| `smallest` \| `spread` \| `random` |
| `TRIWILD_SEED` | 0 | for `TRIWILD_SAMPLE=random` |
| `TRIWILD_REPORT_ONLY` | unset | regenerate the report and exit |

`TRIWILD_MEM_GB` is a cap **per model**, not a budget for the sweep — 16 × 128 G is far
more than any machine has. It is a runaway-killer, and it binds harder in 2D than in 3D
because the dataset's files span 2.7 KB to 1.6 GB and the large ones are read whole
before anything else happens.

`TRIWILD_PARALLEL × TRIWILD_THREADS` should be near the core count. The trade is not
neutral: triwild's preprocessing (input simplification, then the 2D arrangement) is
serial, so extra threads only help the optimization loop, while extra concurrent models
contend for memory bandwidth and slow the serial phases for everyone. Measured on 228906,
preprocessing took 32m16s at 32×4 and 23m19s at 13×8.

## What the sweep fixes about the defaults

Only the knobs the harness itself needs are pinned; everything else comes from the spec
defaults of whatever branch `build/` was built from, which is the point — the sweep tests
the defaults rather than a private configuration. Pinned:

- `filter: none` and `skip_winding_number: true`. With `filter: none` the winding number
  only feeds the MSH group tags, which the sweep does not read, and it is brute-force
  O(#queries × #segments) in 2D: libigl has a hierarchical accelerator for 3D triangles
  and none in 2D. It was **95% of one run's failures** — 232 of 245 timeouts died in that
  phase with the mesh already converged.
- `write_vtu: false`. The `.msh` is the real output; the `.vtu` would be written only to
  be pruned.
- `DEBUG_hausdorff: true`, because the two deviation directions are the point of the run.

## Reports

`report/` gets three files:

- `summary.txt` — the same figures as the console output
- `results.csv` — one row per model: time, iterations, energies, element counts, both
  Hausdorff directions, input size, simplification ratio, failure reason
- `index.html` — self-contained HTML: headline stats, the containment invariant,
  distributions, run time vs input size, slowest models, failures. No CDN, no build step.

### Reading the two Hausdorff numbers

They are not interchangeable, and conflating them once made ~60% of models look like
violations:

- **containment**, d(output → input) — *the envelope invariant*. Every point of the output
  must lie within eps of the input. Anything above 1.0 × eps is a real violation.
- **coverage**, d(input → output) — *diagnostic, bounded by nothing*. The simplification is
  allowed to remove detail and the arrangement may drop segments, so a large value means
  the output no longer covers part of the input. Use `DEBUG_euler` to see which curves
  were lost.

Runs made before the direction was corrected stored coverage under the name `hausdorff`
and have no `coverage` key. The report detects them by that absence, labels them
`metric=legacy-coverage`, and keeps them out of the containment statistics rather than
pooling two different quantities.

## Looking at one result — `visualize_triwild.py`

The sweep says *which* model is interesting; this says *why*. It opens a
[polyscope](https://polyscope.run) window with three independently toggleable layers.

```sh
python3 -m venv .venv && ./.venv/bin/pip install polyscope meshio numpy

./.venv/bin/python visualize_triwild.py runs/full/success/10433   # a sweep output dir
./.venv/bin/python visualize_triwild.py input.obj output.msh      # explicit pair
./.venv/bin/python visualize_triwild.py output.msh                # mesh alone
```

| layer | colour | what it is |
|---|---|---|
| input curves | orange | the `.obj` segment network handed to triwild |
| envelope curves | blue | the **simplified** curves, read back out of the `.msh` |
| output mesh | grey | the triangulation |

Orange and blue rather than the usual red and green, so the two curve networks stay
separable under the common colour vision deficiencies.

Given a directory it finds the mesh (`output.msh`, `out.msh`, or the only `*.msh`), then
the input: an `input.obj` beside it, else the path recorded in that run's `config.json` —
which is what lets it run on a sweep `success/<id>/` directory unchanged, where the input
still lives back in the dataset. Without an input you get the other two layers.

The mesh also carries a per-face **AMIPS2D energy** quantity and an `inverted` flag, since
that is usually the reason for opening it at all. The energy is transcribed from
`wmtk::AMIPS2D_energy`; a startup assertion pins an equilateral triangle at the floor of
2, and on every model tried the maximum reproduces that run's `report.json` `max_energy`
exactly. Note that `report.json` and `out.log` show `1e+50` for an inverted or degenerate
triangle — `MAX_ENERGY`, substituted by `get_quality` — whereas the viewer shows the raw
expression and flags the orientation separately, so a broken triangle stays a number you
can look at.

### Two things about the data worth knowing

- **The `.msh` node array is not the mesh.** Gmsh entities own their nodes, so a triwild
  output holds two disjoint blocks: the dim-2 entity's nodes are the mesh vertices, and
  the dim-1 `EnvelopeSurface` entity's nodes are the simplified input curve — whose points
  sit exactly on input `.obj` vertices. Passing the whole array to a surface-mesh
  constructor draws the mesh plus several hundred unreferenced stray vertices.
- **An OBJ `l` record is a polyline, not a segment** — *n* indices mean *n*−1 segments.
  This dataset uses both forms, so reading one segment per record silently drops most of
  the network on the files that use long polylines.

The view opens looking straight down, orthographic, with drag to pan and scroll to zoom;
an **allow rotation** checkbox unlocks turntable navigation and **top view** snaps back.
(The data is flat in *z* = 0, so a *z*-up camera would sit in that plane and show the mesh
edge-on as a line.)

What it deliberately does **not** show is which output *edges* are constrained — tracked
to the curves. The `.msh` does not tag them, and inferring them by proximity would be a
guess presented as data.

## Caveat on comparing runs

The report pools whatever is in `success/`, so a directory filled across several builds or
several `stop_energy` targets yields statistics that mix them. The 19,686-model run of
August 2026 is exactly that case: most of it ran at `stop_energy` 20 and the tail at 100,
which is why its median max energy is 19.8. For a quotable dataset, run the whole corpus
once on one build.
