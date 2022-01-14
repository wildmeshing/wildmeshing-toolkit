# Schedule API

# API proposal v1
See [api](src/wmtk/ExecutionScheduler.hpp)

# Open Questions
- [x] How to avoid tabulating the map from `enum` to operations?
- [ ] How to avoid ifs in the `find_locker`
- [x] `renew` has several options. Type I are interior edges/faces (newly introduced by the said operation), Type II are locally boundary edges/faces which the operation did not modify but would replace with new tuples (may or may not already in the queue already).
    - No Push: this invalidates type II. not expected behavior.
    - Only renews type II:
        - Simply push them: results in duplicated but valid tuples for the same edge. (on both sides).
        - Check for existence in queue should be done through `edge_attributes` (esp. qslim Garland code did the check).
    - All: Push each I + II from new elements: for II, same duplicates issue.
- [ ] The region to lock in different cases depend not only on the operation, but also the energy (qslim). In addition, for mixed operations, the lock region should be decided differently.
- [ ] Could the lock be inside pre-post-hooks: use RAII?
- [ ] Collapse can be symmetric (place at middle point) or not.
  - Either `get_directed_edges()`, and `push_directed_edges()`

## Case Study
- [ ] Split each edge once:
    - TODO: there should be a unit test, which illustrate the `renew` discussion above.
- [x] Shortest Edge Collapse
    - Almost already current interface.
    - [ ] Should push I+II. Currently buggy.
- [ ] Special Case: QSlim-queue? 
    - ? update quadrics for two-ring.
    - In parallel: how much to lock?
- [x] Quality improving swaps: both edge and face 
    - Overall good.
    - Allows for mixed operation.
    - [ ] re-push I+II, and mutex marking should be dynamically decided.
- [x] Vertex Smoothing (in both TetWild and Harmonic)
    - Maybe, or maybe not a different partition.
    - [ ] TODO: one-ring mutex marking.
- [BK04] Remeshing

## Termination