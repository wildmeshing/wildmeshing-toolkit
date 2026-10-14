# simwild sweep scripts

Batch-run simwild over a large 3D dataset, collect per-model results, and report.
Adapted from [tetwild's sweep scripts](../../../../../tetwild/wmtk/components/tetwild/scripts/)
and deliberately the same contract, environment and output layout — only the
`TETWILD_*` variables become `SIMWILD_*`, so a simwild and a tetwild sweep can run
side by side on the same machine without either one's settings leaking into the other.

Written for Thingi10K on `kirby.cs.nyu.edu`, but nothing here is kirby-specific beyond
the default paths. Each model is a single closed surface mesh; simwild's default
`tag_from_winding_number: true` tags the arrangement's one cell inside/outside by
winding number, so no curve network or pre-tagged `.msh` is needed — the same
single-input shape tetwild's Thingi10K sweep uses.

| file | what it is |
|---|---|
| `run_simwild_sweep.py` | the driver: one simwild run per model, capped in time and memory, then the report |
| `sweep.sh` | tmux wrapper — start / stop / attach / status / report |

## Layout

Everything hangs off one root, `SIMWILD_ROOT` (default `/u/3/daniele/simwild-thingi10k-sweep`
— a separate root from tetwild's sweep so the two never share a success/failure namespace):

```
<root>/build/app/wmtk_app     the binary under test
<root>/data/                  the dataset (.stl .obj .ply .off)
<root>/runs/<name>/           output: success/  failure/  .work/  report/
```

A model that exits 0 lands in `success/<id>/`, anything else in `failure/<id>/` with a
`status.txt` recording the reason. Both keep the config, the log and the `report.json`.

## Running

```sh
export SIMWILD_ROOT=/path/to/sweep
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
SIMWILD_ROOT=... SIMWILD_PARALLEL=15 SIMWILD_THREADS=16 \
SIMWILD_JOB_TIMEOUT=21600 python3 run_simwild_sweep.py
```

### Environment

| var | default | notes |
|---|---|---|
| `SIMWILD_ROOT` | the kirby path | `build/`, `data/`, `runs/` live here |
| `SIMWILD_OUT` | `<root>/runs/full` | output directory |
| `SIMWILD_PARALLEL` | 8 | models at once |
| `SIMWILD_THREADS` | 8 | threads per model |
| `SIMWILD_JOB_TIMEOUT` | 10800 | seconds per model |
| `SIMWILD_MEM_GB` | 128 | per model, 0 disables |
| `SIMWILD_LIMIT` | 0 | process at most N new models |
| `SIMWILD_SAMPLE` | `smallest` | `name` \| `smallest` \| `spread` \| `random` — `name` spreads the expensive models through the run instead of stacking them at the end |
| `SIMWILD_SEED` | 0 | for `SIMWILD_SAMPLE=random` |
| `SIMWILD_REPORT_ONLY` | unset | regenerate the report and exit |
| `SIMWILD_REPORT_BANNER` | unset | note shown at the top of the HTML report |

`SIMWILD_MEM_GB` is a cap **per model**, not a budget for the sweep — 8 × 128 G is more
than most machines have. It is a runaway-killer: it stops one pathological mesh from
swapping a shared box, and does nothing in the normal case.

The sweep pins only `eps_rel` (2e-3, simwild's own spec default) and `num_threads`.
Unlike tetwild, there is no `filter` knob to pin: a single-input run is tagged
inside/outside by winding number (`tag_from_winding_number` defaults to `true`),
not by a post-insertion outside-removal pass. Everything else comes from the spec
defaults of whatever branch `build/` was built from, which is the point: the sweep
tests the defaults rather than a private configuration.

## Reports

`report/` gets three files:

- `summary.txt` — the same figures as the console output
- `results.csv` — one row per model: time, iterations, energies, element counts, failure reason
- `index.html` — self-contained: headline stats, distributions, run time vs input size,
  failures by reason, slowest models. No CDN, no build step.

## Reading the results

- **Final max energy is the raw AMIPS3D value**, whose floor for a regular tet is 3
  (`get_max_avg_energy` returns `cbrt(m_quality)` and `m_quality` stores the cube).
  `stop_energy` defaults to 10 for simwild, so a converged model typically lands just
  under it.
- **A success above `stop_energy` exited on `max_iterations`, not on quality.** Those
  are worth looking at separately; they were still improving when the loop ended.
- **`killed by signal 6 (SIGABRT)` usually means an unreadable input**, not a crash:
  simwild reports the defect and throws, and the throw reaches `terminate`. These are
  corpus defects (truncated STL headers, non-numeric coordinates, files with no faces)
  and will fail on any build.
- **OOM rows are killed at `SIMWILD_MEM_GB`**, not by the timeout, and the report
  buckets them separately for that reason.

## Caveat on comparing runs

The report pools whatever is in `success/`, so a directory filled across several builds
yields statistics that mix them. For a quotable dataset, run the whole corpus once on
one build.
