# triwild sweep scripts

Batch-run triwild over a large 2D dataset, collect per-model results, and report.
Written for the 20k 2D curve dataset on `kirby.cs.nyu.edu`, but nothing here is
kirby-specific beyond the default paths.

| file | what it is |
|---|---|
| `run_triwild_sweep.py` | the driver: one triwild run per model, capped in time and memory, then the report |
| `sweep2d.sh` | tmux wrapper — start / stop / attach / status / report |
| `failure_report.py` | post-hoc: where each failed model died, how far it got, at what energy |

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

## Caveat on comparing runs

The report pools whatever is in `success/`, so a directory filled across several builds or
several `stop_energy` targets yields statistics that mix them. The 19,686-model run of
August 2026 is exactly that case: most of it ran at `stop_energy` 20 and the tail at 100,
which is why its median max energy is 19.8. For a quotable dataset, run the whole corpus
once on one build.
