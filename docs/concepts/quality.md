# Quality

tetwild, triwild and simwild improve the shape of their elements after the
[embedding](embedding.md) has produced a valid but badly shaped mesh. They measure the shape of an
element with the AMIPS energy, and apply splits, collapses, swaps and smoothing until the worst
element is good enough. This page describes the energy, the optimization loop, smoothing, what
happens when the loop stops making progress, and the optional coarsening pass at the end. The
loop is shared by the three components (and topological_offset), in
[`TetOptimizerMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.cpp)
and
[`TriOptimizerMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TriOptimizerMesh.cpp).

## The AMIPS energy

Let \(J\) be the Jacobian of the affine map that takes the regular element (the equilateral
triangle, the regular tetrahedron) to the element. The AMIPS energy is

\[
E_{2D} = \frac{\operatorname{tr}(J^\top J)}{\det J},
\qquad
E_{3D} = \frac{\operatorname{tr}(J^\top J)}{(\det J)^{2/3}}.
\]

It does not depend on the element's size, position or rotation. Its minimum is the dimension: 2
for an equilateral triangle, 3 for a regular tetrahedron. It grows without bound as an element
degenerates, and an inverted or flat element has no energy: the code stores a sentinel value
instead, `MAX_ENERGY` (\(10^{50}\); in 3D the logs show its cube root, \(4.6 \cdot 10^{16}\)).

The energy is evaluated by expressions generated for the purpose, in
[`src/wmtk/utils/AMIPS.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/AMIPS.cpp)
and
[`AMIPS2D.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/AMIPS2D.cpp).
The 3D mesh stores the cube of the energy per tetrahedron and falls back to rational arithmetic
when the double result is out of range
([`AMIPS.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/AMIPS.h)):

```cpp
template <typename rational>
double AMIPS_energy_stable_p3(const std::array<double, 12>& T)
{
    auto res = AMIPS_energy(T);

    if (res < 1e8 && res > 2) {
        return std::pow(res, 3);
    } else {
        return AMIPS_energy_rational_p3<rational>(T);
    }
}
```

The *max energy* in the logs and the reports is the largest \(E\) over the mesh (in 3D, the cube
root of the stored value).

## The target: `stop_energy`

[`stop_energy`](../reference/tetwild.md#stop_energy) is the target: the optimization stops when
the max energy is below it. The default is 100 in tetwild and triwild, 10 in simwild. The same
number is not the same target in 2D and 3D, because the floors differ: 100 is 50 times the 2D
minimum and about 33 times the 3D one. simwild can set a different target per region with
[`quality_field`](../reference/simwild.md#quality_field); the optimization then compares each
element's energy with its own region's target.

## The optimization loop

Each iteration runs the four kinds of operation over the whole mesh:

1. **Split** every edge longer than \(\tfrac43\,l\,s\), where \(l\) is the target edge length
   ([`length_rel`](../reference/tetwild.md#length_rel) times the bounding box diagonal) and \(s\)
   the sizing scalar of the edge (the mean of its endpoints', 1 unless refined).
2. **Collapse** edges shorter than \(\tfrac45\,l\,s\). A collapse is refused if it inverts an
   element or raises the worst energy around the edge.
3. **Swap** edges and faces: 3-2, 4-4, 5-6 and 2-3 in 3D, edge swaps in 2D. A swap is taken when
   it lowers the worst energy of the elements it replaces.
4. **Smooth** every vertex (below).

By default ([`interleaved_smoothing`](../reference/tetwild.md#interleaved_smoothing)) each
topology pass is followed by its own smoothing pass (split and smooth, collapse and smooth, swap
and smooth), and the iteration ends early once the max energy is below `stop_energy`. Before the
first iteration, a collapse pass without the length limit and a swap pass remove what the
embedding left redundant; after the last, a collapse and a swap pass run once more.

The loop stops when the max energy is below `stop_energy` and every vertex has double
coordinates (see [Robust geometry](robust-geometry.md#exact-and-rounded-positions)), or after
[`max_iterations`](../reference/tetwild.md#max_iterations) (80).

## Smoothing

Smoothing moves one vertex at a time, keeping the connectivity. For an interior vertex it
minimizes the sum of the AMIPS energies of the elements around it, over the vertex's position,
with Newton's method: polysolve's dense Newton solver, at most 10 iterations, with a line search
that refuses a step that would invert an element
([`SmoothVertex.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/optimization/SmoothVertex.hpp),
[`solver.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/optimization/solver.hpp)).

A vertex on the tracked surface must also stay on the input.
[`smoothing_mode`](../reference/tetwild.md#smoothing_mode) chooses how:

- **`projected`** (the default). Smooth as if the vertex were interior, then walk back towards its
  start, at \(t = 1, \tfrac12, \tfrac14, \dots\) of the step, projecting each candidate onto the
  input. The first projected candidate that inverts nothing and strictly lowers the worst
  incident element is taken; if none is, after
  [`project_line_search_steps`](../reference/tetwild.md#project_line_search_steps) (12) halvings,
  the vertex does not move. The vertex lands exactly on the input.
- **`exact`**. Minimize, with the same Newton solver, the energy below, in which the distance to
  the input pulls the vertex back. The vertex rests at a small distance from the input,
  proportional to \(w\).

The energy of the `exact` mode, for a vertex at \(x\), is

\[
w \sum_{c \ni x} E(c) \;+\; (1 - w)\left(\frac{d(x)}{\varepsilon}\right)^2 ,
\]

where the sum runs over the elements around the vertex, \(d(x)\) is the distance to the input,
taken with its true Hessian (zero in the directions along a flat part of the input, so the vertex
slides freely there), and \(w\) is [`w_amips`](../reference/tetwild.md#w_amips) (\(10^{-4}\)).

Whatever the mode, the new position is kept only if every surface face around the vertex is
still inside the envelope, no element around it is inverted (exact test), and the worst element
around it is not worse than before. Otherwise the vertex stays where it was.

[`num_smoothing_passes`](../reference/tetwild.md#num_smoothing_passes) sets the number of passes
when smoothing is not interleaved. With threads, smoothing runs by colour classes of non-adjacent
vertices (see [Scheduling](scheduling.md#parallel-passes)).

## When the energy stalls: `stuck_refine_*`

Splits, collapses, swaps and smoothing can reach a configuration none of them improves: a
sliver whose edges are too short to split and whose collapse or swap would make something worse.
The max energy then stops going down. After each iteration the loop checks for this with
([`TetOptimizerMesh.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.h)):

```cpp
    virtual bool optimization_stalled(double prev, double cur)
    {
        return (prev - cur) <= m_params.stuck_refine_stall_eps * (cur - optimization_stop_metric());
    }
```

that is, the last iteration improved the max energy by at most
[`stuck_refine_stall_eps`](../reference/tetwild.md#stuck_refine_stall_eps) (0.1) of the distance
still to cover to `stop_energy`. When it did, the loop refines around the worst elements:

- It selects the worst elements: every element whose energy is above
  \(\min(\max(E_{\max}/100,\ \texttt{stop\_energy}),\ 100)\), or the
  [`stuck_refine_num_worst`](../reference/tetwild.md#stuck_refine_num_worst) worst if that is
  positive.
- It multiplies the sizing scalar of their vertices, and of
  [`stuck_refine_rings`](../reference/tetwild.md#stuck_refine_rings) rings around them, by
  [`stuck_refine_factor`](../reference/tetwild.md#stuck_refine_factor) (0.5), down to
  [`stuck_refine_min_scalar`](../reference/tetwild.md#stuck_refine_min_scalar), and grades the
  change into the surroundings so that neighbouring sizings differ by at most
  [`stuck_refine_gradation`](../reference/tetwild.md#stuck_refine_gradation). The smaller target
  length lets the next split pass refine there.
- With [`stuck_refine_force_split`](../reference/tetwild.md#stuck_refine_force_split) (on), it
  also splits the longest edge of each worst element once, whatever its length.

[`stuck_refine_cooldown`](../reference/tetwild.md#stuck_refine_cooldown) (1) skips that many
iterations after a refinement before the next one, so the operations act on the new sizing first.

## The coarsening pass

The collapse in the loop judges a candidate on the geometry right after the collapse, before any
smoothing, so the converged mesh keeps vertices it does not need. The optional
[`coarsen_pass`](../reference/tetwild.md#coarsen_pass) runs after the loop, and removes them
without letting the max energy rise:

- It takes each collapse without the quality check, smooths the vertices within
  `coarsen_smooth_ring` (1) of the merged vertex `coarsen_local_smoothing_passes` times (0 by
  default), and keeps the collapse only if the worst element in that region is no worse than
  before. Elements outside the region are untouched, so the max energy of the mesh cannot rise.
- It alternates such a collapse pass with `coarsen_global_smoothing_passes` (1) ordinary
  smoothing passes, for at most `coarsen_max_rounds` (2) rounds, and stops early when a round
  accepts nothing.
- [`coarsen_unbounded`](../reference/tetwild.md#coarsen_unbounded) (on) lets it coarsen past the
  target edge length, as far as the energy allows. Off, it leaves edges of \(\tfrac45\,l\) and
  longer alone.

The pass trades run time for elements: it is off by default. With
[`volume_history_output`](../reference/tetwild.md#volume_history_output) it is refused when
`coarsen_local_smoothing_passes` is above 0.

## See also

- The [tetwild](../tutorials/tetwild.md), [triwild](../tutorials/triwild.md) and
  [simwild](../tutorials/simwild.md) tutorials, for the settings on real inputs.
- [`OptimizerParameters.h`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/OptimizerParameters.h),
  where every parameter of the loop is documented with the measurements behind its default.
- Hu et al. 2018, [Tetrahedral Meshing in the Wild](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf),
  and Hu et al. 2019, [TriWild](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf).
