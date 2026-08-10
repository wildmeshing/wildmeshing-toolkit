# simwild: behaviours adopted from tetwild / triwild

tetwild and triwild are the mature, heavily-measured applications; simwild is a fork of them
that drifted. When de-duplicating, the rule is: **wherever the copies disagree, the
tetwild/triwild implementation wins, and tetwild/triwild output must not move.** simwild output
may move, and every time it does the reason is recorded here so the simwild side can be
re-checked later.

Columns: what simwild did, what it does now, which copy won, and what actually moved.

---

## Stage 1 — divergence defects

### 1a. `SimWildMeshTri::swap_all_edges` returns the success count

| | |
|---|---|
| **was** | `return true;` — i.e. always 1, whatever happened |
| **now** | returns `total_success` from `run_localized_to_convergence`, and the pass uses `parallel_collect_edge_ops` + localized retry |
| **source** | triwild `EdgeSwapping.cpp::swap_all_edges`, which carries the fix and the note |
| **why it matters** | `local_operations` stops the swap loop when a pass changes nothing (`if (cnt_success == 0) break;`). Returning a constant 1 made that early exit unreachable. |
| **measured** | **No output change** on any of the 8 simwild 2D configs. The loop is bounded by `ops[2]`, which is 1 in every current config, so there was never a second round for the early exit to skip. The defect is real but latent — like the split fallback in #1000, it needs a config that asks for more than one swap round per iteration. |
| **also adopted** | `parallel_collect_edge_ops` and `run_localized_to_convergence`, which simwild-2D used nowhere. Output-neutral here because the pass converges in one round on these inputs. |
