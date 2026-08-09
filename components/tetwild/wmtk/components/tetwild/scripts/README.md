# tetwild sweep scripts

Batch-run tetwild over a large 3D dataset, collect per-model results, and report. The
3D counterpart of [triwild's](../../../../../triwild/wmtk/components/triwild/scripts/),
and deliberately the same contract, environment and output layout — only the `TRIWILD_*`
variables become `TETWILD_*`, so a 2D and a 3D sweep can run side by side on the same
machine without either one's settings leaking into the other.

Written for Thingi10K on `kirby.cs.nyu.edu`, but nothing here is kirby-specific beyond
the default paths.

| file | what it is |
|---|---|
| `run_tetwild_sweep.py` | the driver: one tetwild run per model, capped in time and memory, then the report |
| `sweep.sh` | tmux wrapper — start / stop / attach / status / report |

## Layout

Everything hangs off one root, `TETWILD_ROOT` (default `/u/3/daniele/thingi10k-sweep`):

```
<root>/build/app/wmtk_app     the binary under test
<root>/data/                  the dataset (.stl .obj .ply .off)
<root>/runs/<name>/           output: success/  failure/  .work/  report/
```

A model that exits 0 lands in `success/<id>/`, anything else in `failure/<id>/` with a
`status.txt` recording the reason. Both keep the config, the log and the `report.json`.

## Running

```sh
export TETWILD_ROOT=/path/to/sweep
./sweep.sh full          # every model
./sweep.sh quick         # 100 models spread across the size range
./sweep.sh status        # progress for every run
./sweep.sh stop          # clean stop: drain, write the report, exit
./sweep.sh attach        # attach to the tmux session
./sweep.sh report [full] # regenerate the report without running anything
```

**Resuming is just re-running the same subcommand** — models already in `success/` or
`failure/` are skipped. A clean `stop` abandons in-flight models *unpublished*, so they
are reprocessed on resume rather than being recorded as spurious failures. To retry a
subset under new settings, move those directories out of `success/` or `failure/` first;
anything left in either is treated as already done.

Or drive the runner directly:

```sh
TETWILD_ROOT=... TETWILD_PARALLEL=15 TETWILD_THREADS=16 \
TETWILD_JOB_TIMEOUT=21600 python3 run_tetwild_sweep.py
```

### Environment

| var | default | notes |
|---|---|---|
| `TETWILD_ROOT` | the kirby path | `build/`, `data/`, `runs/` live here |
| `TETWILD_OUT` | `<root>/runs/full` | output directory |
| `TETWILD_PARALLEL` | 8 | models at once |
| `TETWILD_THREADS` | 8 | threads per model |
| `TETWILD_JOB_TIMEOUT` | 10800 | seconds per model |
| `TETWILD_MEM_GB` | 128 | per model, 0 disables |
| `TETWILD_LIMIT` | 0 | process at most N new models |
| `TETWILD_SAMPLE` | `smallest` | `name` \| `smallest` \| `spread` \| `random` — `name` spreads the expensive models through the run instead of stacking them at the end |
| `TETWILD_SEED` | 0 | for `TETWILD_SAMPLE=random` |
| `TETWILD_REPORT_ONLY` | unset | regenerate the report and exit |
| `TETWILD_REPORT_BANNER` | unset | note shown at the top of the HTML report |
| `TETWILD_SANITY_CHECKS` | unset | run with `DEBUG_sanity_checks` — exact-rational orientation checks. Slow, but the only way an inverted tet coming out of the arrangement is named in the log |

`TETWILD_MEM_GB` is a cap **per model**, not a budget for the sweep — 8 × 128 G is more
than most machines have. It is a runaway-killer: it stops one pathological mesh from
swapping a shared box, and does nothing in the normal case. It is not hypothetical.
On Thingi10K 338910 the cap fires for a real reason: that model leaves insertion with
**187 M faces and 108 M edges** from an input that produced only 796 k Delaunay tets,
and `get_faces()` materialises the face list before the surface-marking loop. Measured
peak was 180 GB and still climbing when the run was abandoned, against 81 GB at the
moment insertion finished — so the arrangement is not the cost, the post-insertion face
pass is.

The sweep pins only `eps_rel` (1e-3), `filter` (none) and `num_threads`. Everything
else comes from the spec defaults of whatever branch `build/` was built from, which is
the point: the sweep tests the defaults rather than a private configuration.

## Reports

`report/` gets three files:

- `summary.txt` — the same figures as the console output
- `results.csv` — one row per model: time, iterations, energies, element counts, failure reason
- `index.html` — self-contained: headline stats, distributions, run time vs input size,
  failures by reason, slowest models. No CDN, no build step.

The 3D report carries no Hausdorff section because the sweep does not enable
`DEBUG_hausdorff`; the 2D one does, and reports containment and coverage separately.

## Reading the results

- **Final max energy is the raw AMIPS3D value**, whose floor for a regular tet is 3
  (`get_max_avg_energy` returns `cbrt(m_quality)` and `m_quality` stores the cube).
  `stop_energy` defaults to 100, so a converged model typically lands just under it —
  a median around 87 is the expected shape, not a warning sign.
- **A success above `stop_energy` exited on `max_iterations`, not on quality.** Those
  are worth looking at separately; they were still improving when the loop ended.
- **`killed by signal 6 (SIGABRT)` usually means an unreadable input**, not a crash:
  tetwild reports the defect and throws, and the throw reaches `terminate`. On
  Thingi10K that accounts for 8 models — truncated STL headers, non-numeric
  coordinates, files with no faces. They are corpus defects and will fail on any build.
- **OOM rows are killed at `TETWILD_MEM_GB`**, not by the timeout, and the report
  buckets them separately for that reason.

## Caveat on comparing runs

The report pools whatever is in `success/`, so a directory filled across several builds
yields statistics that mix them. For a quotable dataset, run the whole corpus once on
one build.
