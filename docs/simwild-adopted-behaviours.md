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

### 1b. `SimWildMeshTri` reports why smoothing was refused

| | |
|---|---|
| **was** | `m_smooth_rejects` declared and passed to `smooth_vertex_2d`, but never `reset()` and never logged |
| **now** | reset at the top of each smoothing pass, `to_string()` logged at the end, as triwild does |
| **source** | triwild `Smooth.cpp::smooth_all_vertices` |
| **why it matters** | The counters exist precisely to catch silent failure — `SmoothVertex.hpp:222-228` records tetwild smoothing ZERO vertices for entire runs without anything noticing. Accumulating them across passes and never printing them defeats that. |
| **measured** | No output change; diagnostics only. |

### 1c. simwild-3D skips the Voronoi search for an un-rounded vertex

| | |
|---|---|
| **was** | `if (m_voronoi_split_fn)` |
| **now** | `if (m_voronoi_split_fn && m_vertex_attribute[v_id].m_is_rounded)` |
| **source** | the 2D twin, `SimWildMeshTri::split_edge_after`, which already carried the guard and the reasoning |
| **why it matters** | When the rounded midpoint inverts a tet, the split falls back to the exact rational midpoint. The Voronoi bisection then searches only *doubles* — including the plain double midpoint it reverts to on failure, which is the position that just inverted. Neither tetwild nor triwild has a Voronoi split, so this is simwild-internal: the 2D copy was right and the 3D one had not been updated. |
| **measured** | No output change: no current config sets `m_voronoi_split_fn` and reaches the fallback. |

### 1d. `SimWildMesh::swap_edge_56_before` stores the max energy it computes

| | |
|---|---|
| **was** | computed `max_energy` over the incident tets and dropped it, leaving `cache.max_energy` holding whatever the previous swap op wrote |
| **now** | `swap_cache.local().max_energy = max_energy;` |
| **source** | tetwild `EdgeSwapping.cpp:756` |
| **why it matters** | `swap_cache` is one thread-local shared by the 3-2 / 4-4 / 5-6 ops. Harmless only as long as `swap_edge_56_after` never reads it — but the sibling `swap_edge_44_after` and `swap_edge_after` both do. |
| **measured** | No output change. |

### 1f. simwild-3D routes swap surface detection on the incident-face count

| | |
|---|---|
| **was** | `is_edge_on_surface(t)`, which short-circuits on both endpoints' `m_is_on_surface` flags |
| **now** | `edge_incident_surface_face_count(t) > 0`, ported from tetwild; applied at all three swap sites (3-2, 4-4, 5-6) |
| **source** | tetwild `EdgeSwapping.cpp:144-151` and its comment |
| **why it matters** | A stale `m_is_on_surface` flag makes a genuine surface edge look interior, and swapping it tears the surface. The face-count route consults only the face attributes. |
| **measured** | No output change on the 6 simwild 3D configs — the flags are not stale on these inputs. |
| **not changed here** | simwild still *rejects* surface edges in 4-4 and 5-6 where tetwild allows a surface flip via its generalized `prepare_surface_flip`. That is a larger decision, deferred to the 3D base-class merge. |
