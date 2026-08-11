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

### 1e. The collapse quality gate — NO CHANGE, the premise was wrong

The duplication survey reported "four mutually incompatible spellings" of the quality gate in
`collapse_edge_before`:

```
tetwild     VA[v1_id].m_is_rounded && q > cache.max_energy
simwild-3D  m_collapse_check_quality && VA[v1_id].m_is_rounded && q > cache.max_energy
triwild     VA[v1_id].m_is_rounded && q > m_params.stop_energy && q > cache.max_energy
simwild-2D  VA[v1_id].m_is_rounded && q > target_quality(tid)  && q > cache.max_energy
```

Checked before changing anything, and they are **two** rules, not four — one per dimension,
each correctly adapted in simwild:

* `SimWildMeshTri::target_quality(tid)` **defaults to `m_params.stop_energy`** and is only
  overridden by the per-tag `quality_field`. So simwild-2D's gate *is* triwild's, generalized
  to per-region targets. Replacing it with `stop_energy` would delete a simwild feature, not
  fix a divergence.
* `m_collapse_check_quality` is an extra simwild-only toggle (switched off around `simplify()`),
  not a changed rule. simwild-3D's gate is otherwise tetwild's exactly.

**The real open question is tetwild vs triwild.** tetwild never permits a collapse that worsens
the ring's worst element; triwild permits it as long as the result is still below the target.
They disagree with each other, so "take the tetwild/triwild implementation" cannot resolve it.
Left alone pending a decision and a measurement. See also 1i, the other tetwild/triwild
disagreement.

### 1g. simwild-2D uses the shared `utils::SizingField` helpers

| | |
|---|---|
| **was** | `refine_sizing_around_worst` and `gradation_smooth_sizing` hand-rolled the top-N selection, the BFS region growth, the refinement clamp and the Dijkstra min-relaxation — ~100 lines re-implementing `src/wmtk/utils/SizingField.hpp`, which the other three meshes call |
| **now** | `utils::select_worst_cells`, `grow_vertex_region`, `apply_sizing_refinement`, `gradation_smooth_sizing`; 92 lines deleted |
| **source** | simwild-3D's `refine_sizing_around_worst`, which already had this shape |
| **measured** | No output change on any config. |

**Semantics deliberately preserved, not "corrected" to triwild's.** triwild filters worst cells
on an absolute `filter_energy = min(max(max_energy/100, stop_energy), 100.)`; simwild filters on
**relative** quality against the per-cell `target_quality(tid)` at a threshold of 1.0. That is
simwild's model — its whole stop condition is per-cell relative quality — and simwild-3D already
expresses it that way through the same shared helper. The hand-rolled 2D filter
(`if (q < target_quality(tid)) continue;`) is exactly equivalent to it.

**Still missing in simwild-2D, not addressed here** (feature ports, not de-duplication):
force-split of the worst cells' longest edges (`m_force_split_edges`), which tetwild, triwild and
simwild-3D all have; and `skip_good_regions` / `active_vertices`, which the other three use to
restrict smoothing.

---

## Decisions taken, not applied

Two divergences turned out to be **tetwild vs triwild**, where "take the tetwild/triwild
implementation" has no answer because the two reference apps disagree with each other.

### The collapse quality gate (see 1e) — deferred, decide with measurement

tetwild and simwild-3D never permit a collapse that worsens the ring's worst element; triwild
and simwild-2D permit it while the result stays below the target. Each fork already matches its
own sibling, so nothing is inconsistent *within* a pair. Answering which rule is better is an
optimizer-behaviour question needing a sweep of both variants on the challenging set, not a
de-duplication question. **Left as is.** The 3D/2D base-class merge should surface it as one
named virtual (e.g. `collapse_allows_worsening_below_target()`) so the choice is explicit in one
place instead of implicit in four.

### tetwild's stale `m_posf` after the exact-midpoint split — separate PR

triwild, simwild-3D and simwild-2D all set `p = to_double(m_pos)` after falling back to the
exact rational midpoint; tetwild alone keeps the average of the two endpoint doubles. triwild
documents why its version is better: when an endpoint is itself un-rounded, rounding the exact
midpoint once beats averaging two approximations.

Fixing tetwild necessarily **moves tetwild output**, which is the signal every de-duplication
stage is validated against. **Deferred to its own PR after the refactor**, where a real output
change is the expected result rather than a warning sign.

---

## Stage 1h — `Parameters` struct defaults now match their own spec defaults

Fourteen fields had a C++ struct default that disagreed with the JSON spec default for the same
key, so any code path that builds `Parameters` **without** going through jse — which is what the
unit tests do — ran a different configuration than every driver run:

| app | field | struct was | spec (now both) |
|---|---|---|---|
| tetwild | `eps_rel` | 2e-3 | **1e-3** |
| tetwild | `stop_energy` | 10 | **100** |
| tetwild | `interleaved_smoothing` | false | **true** |
| tetwild | `interleaved_smoothing_passes` | 2 | **1** |
| tetwild | `stuck_refine_num_worst` | 50 | **0** |
| tetwild | `stuck_refine_rings` | 3 | **0** |
| tetwild | `skip_good_regions` | true | **false** |
| triwild | `stop_energy` | 20 | **100** |
| simwild | `skip_simplify` | true | **false** |
| simwild | `eps_simplify_rel` | 2e-3 | **2e-4** (10x) |
| simwild | `preserve_topology` | false | **true** |
| simwild | `stuck_refine_num_worst` | 50 | **0** |
| simwild | `stuck_refine_rings` | 3 | **0** |
| simwild | `skip_good_regions` | true | **false** |

The spec is authoritative: it is what every driver run injects. **No integration output moved**,
which is the expected result — the configs all go through jse. ctest 107/107, so the unit tests
that construct `Parameters` directly either set these explicitly or are insensitive to them.

---

## Stage 2 — extractions

### `report["#t"]` counted faces, not tets, in simwild 3D

| | |
|---|---|
| **was** | `report["#t"] = mesh.get_faces().size();` in `run_3D` — on a `TetMesh` that is the number of **faces** |
| **now** | `mesh.get_tets().size()` |
| **source** | tetwild reports `tet_size()`, triwild reports `tri_capacity()`; both mean *cells* |
| **measured** | The 6 simwild 3D configs report `#t` roughly halved — e.g. `simwild_double_sphere_3d` 22456 -> 10897, a ratio of 2.06, which is what a closed tet mesh gives since each interior face is shared by two tets. **The mesh is unchanged**; only the reported number was wrong. |

`run_2D` was already correct: on a `TriMesh` the faces *are* the cells.

Note for future baselines: simwild 3D `#t` values recorded before this change are face counts.

---

## Stage 3a — simwild-2D adopts `wmtk::TriOptimizerMesh`

`SimWildMeshTri` now derives from the shared 2D base rather than from `wmtk::TriMesh`, and its
copies of the three attribute types, the shared members and 13 methods are deleted in favour of
the base's.

**No simwild output moved, and no behaviour was adopted.** This is the rare case where the policy
("when the two disagree, tetwild/triwild wins") had nothing to decide, because the two copies did
not disagree anywhere. Every difference between simwild-2D's implementation and triwild's was
cosmetic, and each was checked individually before the copy was deleted:

| function | the only difference |
|---|---|
| `is_inverted(array)` | `const auto res` vs `auto res`; `return !(res > 0)` vs an `if/else`; triwild carries a stale comment about `orient3d` that simwild had dropped (dropped here too) |
| `is_inverted(Tuple)`, `is_inverted(size_t)` | simwild inlines the `oriented_tri_vids` temporary |
| `get_quality(Tuple)`, `get_quality(size_t)` | same inlining |
| `get_quality(array)` | none — identical |
| `is_inverted_f` | simwild has only the `size_t` overload; the bodies match. It now also inherits the `Tuple` one |
| `round` | `m_pos << posf[0], posf[1]` vs `to_rational(m_posf)`, which are the same value; both set `m_is_rounded` before the one-ring check. simwild's comment explaining why was the better one and was kept |
| `get_length2` | `get_edge_vids` vs `switch_vertex`, same two vertices |
| `partition_mesh` | `auto i` vs `size_t i` as the loop variable (simwild's compares signed to unsigned) |
| `round_all_vertices`, `get_max_avg_energy`, `gradation_smooth_sizing`, `get_edges_by_condition`, `is_edge_on_surface` x2, `is_edge_on_bbox` x2, `vertex_is_on_surface`, `edge_is_on_surface` | none — identical after normalisation |

The attribute types matched too: `CellTag` is a `std::set<int64_t>`, so `FaceAttributes` was already
character-identical, and `VertexAttributes` differed only by triwild's `m_feature_id`, which
simwild-2D leaves at `NO_FEATURE`.

**Measured:** all 53 registered configs byte-identical against the run immediately after the move
commit — every simwild config included — plus ctest 107/107. `tetwild_crown` and `tetwild_octocat`
run 10 threads and are nondeterministic there, so they were compared separately at `num_threads: 0`
against a build of the pre-refactor commit: byte-identical.

One thing to watch later: `SimWildMeshTri` keeps its own `round_and_check_all_rounded`,
`m_exact_split_count` and `triangle_area`, none of which triwild has. The first two are the
subject of Stage 4's `RationalPositions` mixin.
