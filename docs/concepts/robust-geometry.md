# Robust geometry

The meshing components take inputs as they come: self-intersecting, open, non-manifold,
degenerate. Three mechanisms keep their meshes valid on such inputs: vertices keep exact rational
coordinates where doubles cannot represent them, every orientation test is exact, and an envelope
bounds how far the surface may move from the input.

## Exact and rounded positions

The optimizer meshes (`TriOptimizerMesh` and `TetOptimizerMesh`, behind tetwild, triwild, simwild
and topological_offset) keep each vertex position twice: as doubles (`m_posf`), which everything
reads, and as rationals (GMP, `wmtk::Rational`), which the exact tests read. A flag,
`m_is_rounded`, says which one is the vertex's position.

- **Where rationals come from.** The [embedding](embedding.md) computes an exact arrangement of the
  input with a background mesh. A vertex where the input crosses itself or the background mesh
  has rational coordinates that are in general not doubles.
- **Rounding.** A vertex is *rounded* when its rational position is replaced by its conversion to
  doubles. That happens only if no incident cell becomes inverted or flat; otherwise the vertex
  keeps its rational position. After the embedding, the vertices whose coordinates are already
  doubles are rounded at once, and every other vertex is tried.
- **During the optimization.** Only an edge split creates a rational vertex: it places the new
  vertex at the midpoint in doubles, and if that inverts a cell, at the exact rational midpoint,
  which lies on the edge and cannot invert anything. Collapses, swaps and smoothing never un-round
  a vertex. A collapse tries to round the vertex it keeps, smoothing the vertex it moves, and a
  sweep tries every remaining rational vertex after each smoothing pass.
- **At the end.** The optimization does not stop at its energy target while a vertex is still
  rational (the log says `energy target reached, but ... vertices are still un-rounded;
  continuing`). The output is written in doubles; tetwild logs `Not all vertices rounded!` when
  the run ends with rational vertices left. simwild's
  [`stop_at_float`](../reference/simwild.md#stop_at_float) stops as soon as every vertex is
  rounded, regardless of the energy.

Rounding a vertex in 3D is this
([`TetOptimizerMesh.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/TetOptimizerMesh.cpp),
two lines for dry runs left out):

```cpp
bool TetOptimizerMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) return true;

    const auto old_exact = m_vertex_attribute[i].m_exact;
    m_vertex_attribute[i].set_pos_to_posf();
    auto conn_tets = get_one_ring_tets_for_vertex(v);
    m_vertex_attribute[i].m_is_rounded = true;
    for (auto& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_exact = old_exact;
            return false;
        }
    }

    return true;
}
```

The rational copy is stored only while it differs from the doubles
([`ExactPosition.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/ExactPosition.hpp)),
so a rounded vertex costs nothing extra.

## Exact predicates

Every decision about orientation is exact:

- **Orientation tests** (`orient2d`, `orient3d` in
  [`src/wmtk/utils/predicates.hpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/utils/predicates.hpp))
  use Marco Attene's Indirect_Predicates: a floating-point filter answers when it can decide the
  sign, and exact arithmetic answers when it cannot.
- **Inversion.** A cell whose vertices are all rounded is tested with `orient3d` (or `orient2d`)
  on the doubles; a cell with a rational vertex is tested by computing the determinant in rational
  arithmetic. A flat cell counts as inverted.
- **Quality.** In 3D, the [AMIPS energy](quality.md#the-amips-energy) of a tetrahedron is
  computed in doubles when all its vertices are rounded and the result is in the range doubles
  evaluate reliably, and in rational arithmetic otherwise. In 2D it is computed in doubles.
- **Smoothing** refuses any Newton step that would invert a cell around the vertex, by the same
  orientation tests.

The arrangement that creates the rational vertices is itself exact; see [Embedding](embedding.md).

## Envelopes

The *envelope* is the set of points within distance \(\varepsilon\) of the input. The meshing
components keep their tracked surface (or, in 2D, their tracked curves) inside it: an operation
that creates or moves a surface face checks the new faces, and is refused if one leaves the
envelope. The envelope bounds how far the output strays from the input. It does not bound the
converse: a part of the input the output no longer covers is not detected by it.

\(\varepsilon\) is set relative to the input. [`eps_rel`](../reference/tetwild.md#eps_rel) is a
fraction of the diagonal of the input's bounding box, \(\varepsilon = \texttt{eps\_rel} \cdot
\text{diag}\), and the target edge length [`length_rel`](../reference/tetwild.md#length_rel) is
relative to the same diagonal. simwild also takes absolute values,
[`eps`](../reference/simwild.md#eps) and [`length`](../reference/simwild.md#length).

tetwild builds the main envelope around the original input, at \(\varepsilon\), and the other
envelopes it needs as fractions of it: the input simplification runs inside
`simplify_envelope_ratio` \(\cdot\,\varepsilon\) (0.5), so the optimizer starts with room to move;
open boundaries and non-manifold edges stay in a tube of `order2_envelope_ratio` \(\cdot\,\varepsilon\);
feature curves in one of `feature_envelope_ratio` \(\cdot\,\varepsilon\). With several inputs,
each tracked face is held to the envelopes of the inputs it lies on, not only to the envelope of
all of them together.

### The containment test: exact or sampled

Whether a triangle (or a segment) lies inside the envelope is decided by one of two tests. Both
answer the same question about the same envelope: they are given the same \(\varepsilon\), and
neither is wider than the other. They differ in how they answer.

- **Exact** (the default). The test of fast-envelope
  ([Wang et al. 2020, Exact and Efficient Polyhedral Envelope Containment Check](https://cims.nyu.edu/gcl/papers/2020-Fast-Envelope.pdf)),
  decided with exact predicates. It has three versions: for triangles in 3D, for segments in 3D,
  and for segments in 2D.
- **Sampled** ([`use_sample_envelope`](../reference/tetwild.md#use_sample_envelope)). Points are
  laid on the query: a lattice of spacing \(\varepsilon\) on a triangle, evenly spaced points less
  than \(\varepsilon\) apart on a segment. Each point is tested against the input through a
  bounding-volume hierarchy. Since the test cannot see between two points, it accepts a point
  only within a radius smaller than \(\varepsilon\), reduced by the distance from any point of the
  query to the nearest sample: \(\varepsilon - \varepsilon/\sqrt{3}\) for the lattice on a
  triangle, \(\varepsilon/2\) for points on a segment. If every sample passes, every point of the
  query is within \(\varepsilon\).

The reduced radius is the sampled test's own conservatism, not a smaller envelope. Its cost is
that the sampled test refuses some geometry that is inside the envelope. Its purpose is speed: on
some inputs the exact test dominates the run time. The two tests disagree on geometry close to
the envelope's boundary, so switching from one to the other changes the output.

The radii, from
[`Envelope.cpp`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/src/wmtk/envelope/Envelope.cpp),
where `_eps` is the envelope's \(\varepsilon\) (`eps2` and `eps2_edge` are the squared radii the
samples are tested against):

```cpp
    sampling_dist = _eps;
    const double real_envelope = _eps - _eps / std::sqrt(3);
    eps2 = real_envelope * real_envelope;
    // Edge queries sample a line, not a lattice; see eps2_edge in the header.
    const double real_envelope_edge = _eps - _eps / 2;
    eps2_edge = real_envelope_edge * real_envelope_edge;
```

tetwild, triwild and simwild also have
[`simplify_use_sample_envelope`](../reference/tetwild.md#simplify_use_sample_envelope), which
switches the test for the input simplification only.

## See also

- [Embedding](embedding.md): where the rational coordinates come from.
- [Quality](quality.md): what the operations optimize inside the envelope.
- The [QSlim tutorial](../tutorials/qslim.md) measures how the envelope limits a simplification.
