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

**Still missing in simwild-2D, not addressed here** (a feature port, not de-duplication):
force-split of the worst cells' longest edges (`m_force_split_edges`), which tetwild, triwild and
simwild-3D all have. `skip_good_regions` / `active_vertices` were adopted with the shared 2D
smoothing pass in Stage 5 below.

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

---

## Stage 3b — simwild-3D adopts `wmtk::TetOptimizerMesh`

`SimWildMesh` now derives from the shared 3D base, dropping its copies of `VertexAttributes`,
`FaceAttributes`, the vertex and face attribute collections, the envelope/solver/rounding members,
`MAX_ENERGY`, and 23 method definitions.

Of the 21 methods the base owns, **19 were already byte-identical** between the two applications and
two (`get_quality`, `output_faces`) differed only cosmetically -- `auto` vs `bool` for a local, and an
`(int)` cast on a row assignment into a matrix that is already `int`. That leaves exactly one
behavioural adoption:

### `round()` now marks the vertex rounded before the one-ring check

| | |
|---|---|
| **was** | simwild left `m_is_rounded` false until every incident tet had been checked, so `is_inverted` took the **exact rational** branch throughout the check |
| **now** | tetwild's: `m_is_rounded` is set before the loop and cleared again on failure, so `is_inverted` takes the **float** branch |
| **source** | tetwild, triwild and simwild-2D all already did it this way; simwild-3D was the only one that did not |

**This is answer-preserving, not merely close.** The check asks whether the ROUNDED position keeps
every incident tet valid. Under the old order the vertex's `m_pos` had already been set to
`to_rational(m_posf)` -- exactly the double, as a rational -- so the rational branch evaluated the
orientation of the same four points that the float branch evaluates with `orient3d`, which is itself
exact for the doubles it is handed. Both branches also fall back to rational identically when some
*other* incident vertex is un-rounded, since `is_inverted` tests all four flags. The difference is
cost, not result: simwild was paying for exact arithmetic to compute an answer the float predicate
already decides exactly.

### Not adopted in this commit

The swap family stays per-application and is untouched here: `swap_face_before`, `swap_face_after`,
`swap_edge_after` and `swap_all_faces` all propagate simwild's `CellTag` through the operation, which
tetwild has no counterpart for. Sharing them needs a tag-propagation hook. Three differences found
while measuring them are recorded here so they are not lost:

* **`swap_all_faces` collects the wrong simplex.** simwild calls `parallel_collect_edge_ops` and
  queues `"face_swap"` operations on **edge** tuples -- 6 per tet -- where tetwild calls
  `parallel_collect_face_ops` and queues them on faces, 4 per tet. The face-swap pass therefore
  enumerates a different set, with each face reachable from up to three of its edges.
* simwild calls the executor directly where tetwild uses `run_localized_to_convergence`.
* simwild reads the success count from `executor.get_cnt_success()` after the lambda has returned;
  tetwild accumulates it into a captured local.

At this stage `smoothing_energy_envelope` stayed per-application. Stage 6 below subsequently
removed SimWild's redundant surface-envelope alias and adopted TetWild's order-`>= 2` selection;
only the application-specific order-2 envelope object remains virtual.

`smooth_after` stays in **both** applications even though their bodies are byte-identical: the body
is the shared `optimization::smooth_vertex_3d` template, which reaches `m_tet_attribute` and
`smoothing_energy_envelope`, both per-application.

---

## Stage 3c — simwild adopts `wmtk::run_pass`

The `if (NUM_THREADS > 0)` / build an `ExecutePass` / install a lock / log the time shell around
every driver is now `wmtk::run_pass`, shared by all four meshes. **No simwild behaviour changed**:
the shell was already identical to tetwild's and triwild's in all eleven simwild drivers, and what
each driver does *inside* the executor is passed in as a callback, so the differences below survive
the fold untouched rather than being silently adopted.

They are listed here because folding the shell is what makes them visible, and because each one is
now a one-line change instead of a rewrite:

| | simwild | tetwild / triwild |
|---|---|---|
| how the pass runs | `executor(mesh, ops)` once, in 3D split, all four 3D swaps and 2D split/smooth | `run_localized_to_convergence(mesh, executor, ops)` — retries a failure only where the mesh actually changed that round |
| 2D collapse | a hand-rolled `do { ... } while (executor.get_cnt_success() > 0)` that rebuilds the whole op list from `get_edges()` every round | the same localized retry as everything else |
| success count | `executor.get_cnt_success()`, i.e. the last round's count | the total accumulated across rounds by the retry driver |

The success-count row matters where the caller uses the return value to decide whether to keep
going: `local_operations` stops its swap loop once a pass changes nothing, and a per-round count and
a total disagree about when that is. simwild-2D's swap already takes the shared form (Stage 1a); the
3D swaps do not.

Adopting the localized form is a real behaviour change for simwild -- it changes which failed
operations are re-tested and in what order -- so it wants its own commit and its own before/after,
not a line in a refactor.

---

## Post-3c — simwild adopts the localized retry and the accumulated success count

Closes the three differences Stage 3c recorded but deliberately left alone. Every driver now runs
its pass the way its sibling does.

| driver | was | now |
|---|---|---|
| 3D split, all five 3D swaps, 2D split | `executor(mesh, ops)` — **a single pass**, so an operation rejected early was dropped even when a later success next to it made it viable | `run_localized_to_convergence`, which re-runs until a pass yields nothing, re-queueing only failures adjacent to a vertex that moved that round |
| 2D collapse | `do { all_ops.clear(); for (e : get_edges()) …; executor(…); } while (get_cnt_success() > 0)` — converged, but by re-enumerating every edge and re-running the full geometric pre-check on each one, every round | the same shared retry driver, off an op list built once |
| the five 3D swaps' return value | `executor.get_cnt_success()`, the **last** round's count | the total accumulated across rounds |

The return value is not bookkeeping: `local_operations` stops its swap loop when a pass reports
zero, and a pass that has converged ends on a zero round by construction, so the old form made the
loop stop after one round of swaps regardless of how much work remained.

The 2D collapse needed a `renew_neighbor_tuples` — its rebuild-everything loop never had one, and
the retry driver takes the modified region from exactly that callback. It is triwild's.

**Not adopted here, still open:** simwild builds its op lists with a serial
`for (const Tuple& e : get_edges())` where tetwild and triwild use `parallel_collect_edge_ops`. That
changes the queue's insertion order, which breaks ties in the priority queue, so it is its own
behaviour change and would have blurred this one's before/after.

### Measured

Four configs moved; every triwild and deterministic tetwild config is byte-identical, and the branch
touches no file outside `components/simwild/`.

| config | before | after |
|---|---|---|
| `simwild_curves_2d` | 572 tris, avg 2.385, max 8.294 | 384 tris, avg 2.499, max 8.494 |
| `simwild_curves_two_inputs_2d` | 465 tris, avg 3.343, max 18.862 | 871 tris, avg **2.815**, max 18.844 |
| `simwild_remeshing_2d` | 211 tris, avg 2.299, max 4.330 | 246 tris, avg 2.324, max **4.188** |
| `simwild_double_sphere_notop_3d` | 21454 tets, avg 3.6171 | 21459 tets, avg 3.6170 |

The direction is mixed, and the mechanism is visible in `simwild_curves_2d`'s log. The split pass
now converges instead of running once, so the mesh refines far harder mid-run -- 1955 edges at
iteration 5 where it used to reach 695 -- and the collapse pass then pulls it back further than
before, to 378. The optimizer takes a different trajectory rather than a uniformly finer or coarser
one. Collapse itself is unchanged in aggregate: the first call's rounds used to sum to ~316
successes and the shared driver now reports 313.

---

## Post-3c — simwild adopts `parallel_collect_edge_ops`

The last collection-side difference. simwild built three op lists with a serial
`for (const Tuple& e : get_edges())` -- 3D split, 2D split, 2D collapse -- where tetwild and triwild
use the shared parallel builder. Held back from the localized-retry change because it alters the
queue's *insertion* order, which is what breaks ties between equal-priority operations, and mixing
the two would have made neither before/after attributable.

**Measured: nothing moved.** All 53 registered configs are byte-identical, simwild's four included
-- the same four that moved for the retry adoption. The tie-breaking concern is real in principle
and did not materialize on any config in the suite.

That leaves simwild's op collection identical to its sibling's everywhere except `swap_all_faces`,
which is still `parallel_collect_edge_ops` where tetwild uses `parallel_collect_face_ops` -- that
one is a defect rather than a style difference (6 edge tuples per tet instead of 4 faces) and is
recorded above with the rest of the swap family.

---

## Post-3c — `swap_all_faces` collects faces

The defect recorded with the swap family, fixed on its own because it is a behaviour change rather
than a sharing question.

`SimWildMesh::swap_all_faces` called `parallel_collect_edge_ops` and queued `"face_swap"` operations
on **edge** tuples -- 6 per tet, with each face reachable from up to three of its edges -- where
tetwild calls `parallel_collect_face_ops` and queues them on faces, 4 per tet. The pass therefore
enumerated a different set of simplices from the one it is named for, seeding the priority queue
with duplicates of some faces and a different traversal for the rest.

Two configs move, both of them the 3D ones that actually run face swaps:

| config | before | after |
|---|---|---|
| `simwild_double_sphere_3d` | — | same `#t`/`#v`, different connectivity |
| `simwild_double_sphere_notop_3d` | 21459 tets, avg 3.616959 | 21528 tets, avg 3.617020 |

Everything else in the suite is byte-identical, and the branch touches no file outside
`components/simwild/`.

Both remaining swap differences are settled below.

---

## Post-3c — simwild adopts tetwild's swap family, with tag propagation

The last item in the de-duplication programme, and the largest single behaviour change in it.
`components/simwild/.../EdgeSwapping.cpp` is now tetwild's file: after renaming the mesh class and
the parameters struct, the only lines that differ are the tag handling described below and one
`swap_all_faces` comment. Every remaining divergence has been resolved in tetwild's favour.

### Why tags and the surface are the same question

simwild's surface is not an input mesh — `init_surfaces_and_boundaries` tags a face
`m_is_surface_fs` **exactly when its two tets carry different tags**. So a surface diagonal flip in
simwild *is* a flip of the tag interface, and the volume it sweeps (the tet `(a,b,c,d)` between the
old and new diagonal) moves from one tag to the other. That is the same thing tetwild's flip does
across its inside/outside surface, and it is gated the same way, by the Hausdorff envelope check on
the two new faces.

Two consequences fall out and are used throughout:

- An edge with **no** incident surface face has all its incident tets identically tagged, so an
  interior swap has nothing to decide.
- The two surface faces `(a,b,c)`, `(a,b,d)` cut the ring of incident tets into two arcs, one tag
  each — and every tet a flip creates contains at least one ring vertex other than `c`,`d`, which
  says which arc, hence which tag, it belongs to.

### What simwild adopted

| | before | after |
|---|---|---|
| `prepare_surface_flip` | `prepare_surface_flip_32`: exactly-3 ring only, `log_and_throw_error` on anything unexpected, `assert`s that `(c,d)` already exists | tetwild's, generalized to any ring size; rejects rather than throws; handles `(c,d)` not existing yet, which is the normal case for 4-4 and 5-6 |
| 4-4 and 5-6 on a surface edge | rejected outright (`edge_incident_surface_face_count > 0` → `false`) | steered to the case that realizes the flip, by `swap_edge_44_accept_case` / `swap_edge_56_accept_case`, then envelope-checked and re-tagged like the 3→2 case |
| the `(c,d)` non-manifold guard | `assert` + `get_surface_faces_for_edge` | tetwild's direct enumeration, which does not depend on the possibly-stale `m_is_on_surface` vertex flags |
| the "would-be new faces already surface" guard | absent | tetwild's, gated on `lowest_common_tet` so a Debug build does not abort on every 4-4 flip |
| `swap_edge_56_after` | re-checked `is_inverted` | tetwild's: `swap_edge_56_energy` returns `double::max()` for any inverted candidate and the case is only taken when strictly below the old energy, so the check could never fire |
| `swap_all_edges_44` / `_56` | no `check_surface_topology` guard | tetwild's guard, now that these can change the surface |
| `swap_all_edges_all` | logged its topology warning as `"swap_all_edges_32"` | its own name |
| counters | `cnt_surface_swap` only | plus tetwild's `cnt_surface_swap_32/44/56` breakdown, and the same log line |
| open-boundary edges | not considered | `wmtk::TetOptimizerMesh::is_open_boundary_edge`, a new virtual that defaults to `false` and that tetwild overrides — see below |

`is_open_boundary_edge` defaulting to false is exact for simwild, not a stub: around an interior
edge the tags change an even number of times going around the tet ring, so the interface can never
terminate there; it can only end where it meets the bbox, and `is_edge_on_bbox` has already
rejected those edges by the time the question is asked. Making it a virtual is what leaves the two
`prepare_surface_flip` bodies byte-identical.

### The tag rule

Three simwild-only additions, and nothing else:

- `cache_interior_swap_tag` — no incident surface face, so record the one tag the incident tets
  share. If they *disagree* the swap is refused: that can only happen if the tag/surface invariant
  is already broken elsewhere, and re-tagging would move tagged volume with no envelope check to
  gate it. This replaces the old 3→2 rule (majority vote over at most two tags) and the old 4-4 /
  5-6 rule (take `incident_tets[0]`, with a commented-out check that they all agree).
- `prepare_surface_flip` additionally records `ring_tags`: for each ring vertex other than `c`,`d`,
  the tag of its side.
- `propagate_swap_tags` — interior swap: give every new tet the cached tag. Surface flip: give each
  new tet the tag of the arc its ring vertices identify, and refuse if a tet straddles both arcs or
  touches neither.

On a well-formed mesh this leaves the 3→2 result unchanged: an interior edge has one tag, so the
majority vote agreed with it, and on a surface edge the majority is by construction the arc holding
two of the three tets, which is the arc both new tets land in.

`SwapInfoCache::sf_e` is gone — the third apex of the 3→2 ring, which nothing read.

### Measured

**tetwild and triwild are byte-identical**, on all 53 registered configs — with `tetwild_crown` and
`tetwild_octocat` re-run at `num_threads: 0`, since at their configured 10 they are nondeterministic
by construction — and on all 30 hidden `[challenging]` models (14 tetwild Thingi10K, 16
triwild20k), compared against two independent baselines: the run for #1011 and the run for #1009,
which also agree with each other. The comparison is every `report.json` field except the timing
keys, plus the md5 of every emitted mesh — `out_final.msh`, `out_simplified_input.obj`,
`out_surface.obj` for tetwild and `out.msh`, `out.vtu`, `out_surf.vtu` for triwild.

Two simwild configs move, the only two that run 3D swaps:

| config | before | after |
|---|---|---|
| `simwild_double_sphere_3d` | 10897 tets, 2198 verts, avg 3.80078 | 10888 tets, 2197 verts, avg 3.79623 |
| `simwild_double_sphere_notop_3d` | 21528 tets, 4257 verts, avg 3.61702, max 8.5319 | 21311 tets, 4215 verts, avg 3.59206, max 8.7158 |

Both get slightly smaller at slightly better average energy, and the counters say exactly why:

| config | surface flips before | surface flips after |
|---|---|---|
| `simwild_double_sphere_3d` | 4 | 85 = **4** (3→2) + 69 (4-4) + 12 (5-6) |
| `simwild_double_sphere_notop_3d` | 0 | 89 = 1 (3→2) + 70 (4-4) + 18 (5-6) |

The 3→2 count on the first config is *unchanged*, which is the claim above about the tag rule
being behaviour-preserving there, measured rather than argued. All of the movement is the 4-4 and
5-6 flips that simwild used to refuse outright — the ones that let the optimizer relieve a bad
configuration on the interface instead of routing around it. (`notop`'s 3→2 going 0 → 1 is
downstream: the mesh reaches configurations it never used to.)

Re-running both with `check_surface_topology: true` — which takes a full topological signature of
the tracked surface (V, E, F, Euler, components, boundary loops) before each swap pass and reports
any change afterwards — gives **zero complaints across all 174 flips**. A diagonal flip is supposed
to leave surface topology alone, and on real data it does.

### Tested

`test_simwild_swap_tags.cpp` builds an N-tet ring whose two arcs carry different tags, derives the
surface from the tags the way `init_surfaces_and_boundaries` does, runs the flip, and checks that
every face is surface **iff** its two tets still disagree — for the 3→2, 4-4 and 5-6 flips, plus an
interior swap that must preserve its single tag. The 4-4 and 5-6 cases assert the resulting tets by
vid, so a wrong side would fail rather than merely a wrong count. Release and Debug.

Writing it turned up an unrelated latent defect in simwild's test-only `create_mesh_attributes`,
fixed here: it assigned `m_tet_attribute.m_attributes` a vector sized to the *live* tet count,
shrinking the collection below the connectivity capacity that `init()` had reserved. An
`AttributeCollection` deliberately never grows during an operation, so a 5→6 swap indexed one past
the end. tetwild carried the same bug and had already fixed it (ASan found it there); this is the
same fix, and simwild's version segfaults outright rather than corrupting quietly because the field
being written is a `std::set` and not a `double`.

### 2D was already done

For completeness, since "adopt all the swaps" covers both dimensions: the 2D swap family
(`swap_all_edges`, `swap_weight`, `swap_edge_before`, `swap_edge_after`) is **already identical**
between triwild and simwild-2D — after renaming the mesh class, the only differences are two
comments. Stage 3a's shared `TriOptimizerMesh::FaceAttributes` carries `tags` for both, so even the
tag lines are common code, and `SwapInfoCache::face_tags` is triwild's too. (triwild has carried an
unused `std::set<int64_t> tags` per triangle since well before this programme — see
`git show origin/main:.../TriWildMesh.h` — so that is pre-existing, not something the move added.)

The tag propagation is correct there for a simpler reason than in 3D: the 2D swap opens with
`if (is_edge_on_surface(t)) return false;`, and in 2D too the surface is the tag interface
(`SimWildMeshTri::init_surfaces_and_boundaries`), so a 2D swap only ever runs on an edge whose two
faces already agree. `cache.face_tags = FA[incident_faces[0]].tags` is then exact. There is no 2D
counterpart to the 4-4 / 5-6 question because a 2-2 flip is the only edge swap a triangle mesh has.

---

## Stage 4 — one swap engine, with serial Wild/Sim conformance tests

The identical implementations described above are no longer merely kept in sync:

- TriWild's complete 2D swap pass now lives on `TriOptimizerMesh`. Both `TriWildMesh` and
  `SimWildMeshTri` inherit the same collector, priority calculation, retry/termination logic,
  before/after callbacks, quality gate, attribute tracker and face-tag propagation.
- TetWild's complete 3D swap family now lives on `TetOptimizerMesh`: 3→2, 4→4, 5→6, 2→3 face
  swaps, the combined pass, surface-diagonal handling and topology diagnostics. SimWild's old
  858-line copy is a small adapter implementing only three tag hooks: cache one homogeneous
  interior tag, cache the two surface-ring arcs, and propagate those tags to new cells.
- TetWild's 3D smoothing pass and acceptance callback also live on `TetOptimizerMesh`. The only
  application policy left virtual is which envelope supplies the surface energy/containment
  check. Quality reads and writes go through the common cell-quality accessors, so TetWild's
  pass ordering and rejection rules are used unchanged with SimWild's tagged cell attributes.

`test_wild_conformance.cpp` enforces the intended oracle relationship directly. It constructs two
identical meshes, runs the Wild implementation first and tag-homogeneous SimWild second with
`NUM_THREADS == 0`, then compares operation counts, canonical connectivity, per-cell quality and
vertex positions; the SimWild side must additionally preserve its homogeneous tag. There is one
fixture for TriWild/SimWild 2D and one for TetWild/SimWild 3→2. These are deliberately serial: a
parallel A/B test can turn shared global state or nondeterministic scheduling into noise and does
not establish that SimWild followed the Wild path. A third fixture runs the shared TetWild/SimWild
3D smoothing pass serially and compares the same geometry, exact positions and qualities while
checking that SimWild's cell tags are untouched.

Two audit fixes were made alongside the extraction:

- `stop_at_float` now applies to SimWild 2D as well as 3D.
- the public 3D `replace_tags` rejects mismatched input/output vector lengths, matching its 2D
  counterpart instead of indexing past `found_tags`.

---

## Stage 5 — one 2D smoothing pass

TriWild's complete smoothing pass now lives on `TriOptimizerMesh`, matching the already-shared 3D
pass on `TetOptimizerMesh`. Both `TriWildMesh` and `SimWildMeshTri` inherit the same vertex
collection/order, rounding and bbox gate, two-stage solve, inversion/envelope/quality rejection,
quality updates, rejection counters, and `skip_good_regions` selection. The shared base also owns
the identical floating/exact position reads and writes.

Both applications use the base's single `m_envelope` for the pull energy and containment. Its
geometry is application-specific -- TriWild builds it around its input curves, while SimWild can
build it around the interface between different cell tags -- but there is only one envelope per
2D mesh. The only smoothing acceptance policy left virtual is the positional constraint: TriWild
uses it to preserve feature points and SimWild, which has no separate 0-dimensional features,
always accepts it.

Two SimWild behaviors deliberately move to TriWild's ground truth:

- SimWild no longer sets `quality_veto_on_surface = false`; a surface move that worsens the worst
  incident triangle is rejected in both applications.
- SimWild now honors `skip_good_regions`; a vertex incident only to triangles below
  `skip_good_regions_margin * stop_energy` is not scheduled for smoothing.
- SimWild's unused 2D `m_envelope_orig` alias is deleted. Energy and containment both use
  `m_envelope`, exactly as in TriWild. This does not change the standard remeshing path because
  the two pointers aliased the same object there.

The serial conformance fixture runs TriWild first and tag-homogeneous SimWild second from identical
2D meshes, then compares connectivity, floating and exact positions, per-face qualities, and tags.
It separately enables `skip_good_regions`, makes every face good, and verifies both applications
schedule zero vertices. Full TriWild and SimWild test suites pass.

---

## Stage 6 — envelope and refinement parity

SimWild 3D now has the same envelope model as TetWild: one surface `m_envelope`, plus the separate
order-2 feature envelope. The old `m_envelope_orig` pointer was always assigned from `m_envelope`
and is deleted. Feature pulling now uses TetWild's exact condition, order `>= 2`, including
order-3 junction vertices, and falls back to the surface envelope when no initialized feature
envelope exists.

The remaining refinement gaps are also closed:

- SimWild 2D records the longest edge of each selected worst triangle in `m_force_split_edges`,
  matching TriWild.
- SimWild 3D applies TetWild's oversized-only force-split guard, including the JSON option.
- `active_vertices()` is virtual at the optimizer bases. Wild keeps its absolute global threshold;
  SimWild overrides it by normalizing every cell against its tag-dependent `target_quality()`.

The serial conformance suite now also covers 3D 4→4, 5→6, 2→3, and combined swaps; order-3
feature-envelope selection; 2D/3D force-split refinement; per-tag active-region filtering; and
heterogeneous-tag split/collapse invariants in both dimensions.

---

## Stage 7 — shared outer optimization drivers

The outer schedules now live on the dimensional optimizer bases:

- `TetOptimizerMesh` owns the complete TetWild/SimWild 3D driver and its `local_operations` pass.
- `TriOptimizerMesh` owns the complete TriWild/SimWild-2D driver and its `local_operations` pass.
- the smoothing placement parameters (`num_smoothing_passes`, `interleaved_smoothing`, and
  `interleaved_smoothing_passes`) live in `OptimizerParameters`, so all three components configure
  the same schedule.

TetWild and TriWild remain the behavioral sources of truth: pre- and post-collapse, interleaved or
batched split/collapse/swap/smooth ordering, per-phase convergence checks, rounding, consolidation,
stuck refinement, swap termination, debug output, and sanity checks are executed by their shared
base. The application classes no longer contain copies of either driver.

Only explicit policy hooks differ. Wild uses absolute AMIPS and `stop_energy`; SimWild reports
quality normalized by each cell's tag-dependent target and stops at relative quality 1. SimWild
also supplies its existing optional `stop_at_float` policy and tag-aware refinement, while TetWild
retains its extra open-boundary sanity diagnostic.

The conformance suite now runs one complete iteration under both the batched and interleaved
schedules, serially from identical meshes in each dimension: TriWild then homogeneous SimWild-2D,
and TetWild then homogeneous SimWild-3D. It compares iteration counts, canonical connectivity,
per-cell qualities, floating and exact positions, and checks that SimWild kept its homogeneous tags
and derived interface invariant.
