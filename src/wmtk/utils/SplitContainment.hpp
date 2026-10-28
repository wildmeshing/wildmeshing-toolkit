#pragma once

#include <wmtk/envelope/Envelope.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace wmtk {

/**
 * Whether `p` lies on the segment [a,b], up to the rounding of a point computed on it in double:
 * the midpoint a split places, the exact midpoint rounded once, or a bisection along the edge
 * (simwild's Voronoi placement).
 */
template <typename Vec>
bool lies_on_segment(const Vec& p, const Vec& a, const Vec& b)
{
    const Vec ab = b - a;
    const double l2 = ab.squaredNorm();
    if (l2 == 0) return false;
    const double t = (p - a).dot(ab) / l2;
    if (t < 0 || t > 1) return false;
    const double scale =
        std::max({a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff(), std::sqrt(l2)});
    return (a + t * ab - p).norm() <= 1e-12 * scale;
}

/**
 * Whether the two surface pieces a split leaves behind inherit their parent's containment, rather
 * than being tested again: true when the new vertex lies on the split edge (the caller's part),
 * and the parent and both pieces are judged by the same SAMPLED envelope.
 *
 * The pieces are then subsets of the parent, which is already inside: every surface simplex is,
 * either because an operation tested it or because it is a piece of one that was. Testing them
 * again cannot find a real violation, only a sampling artifact. The sampled test is not
 * hereditary: it places its samples relative to the simplex it is given, so a piece's samples
 * fall where the parent's did not, and one of them can lie beyond the shrunk acceptance radius
 * although every point of the piece is inside the envelope. On the EMI-Meshing surfaces (size
 * 5000, 10 cells, eps 18, acceptance radius 7.61) refused pieces lay 7.6 to 8.5 from the input
 * -- under half the envelope -- while their parents passed.
 *
 * Such a refusal is permanent, the geometry does not change, and it is what hangs a split pass:
 * when the refused edge is the longest of its tets, the next one down is split instead, its new
 * vertex is joined to the far end of the refused edge by an edge about two thirds as long --
 * still due -- and the chain goes on forever, every success renewing the refused edge.
 *
 * The exact envelope is hereditary, so it keeps its test. Containment through an envelope chosen
 * per simplex (topological_offset's tag envelopes) is inherited only when the pieces are judged by
 * the same one as the parent; otherwise they are tested as before.
 */
inline bool split_inherits_containment(
    const std::shared_ptr<SampleEnvelope>& parent,
    const std::shared_ptr<SampleEnvelope>& piece0,
    const std::shared_ptr<SampleEnvelope>& piece1)
{
    return parent && !parent->use_exact && piece0 == parent && piece1 == parent;
}

} // namespace wmtk
