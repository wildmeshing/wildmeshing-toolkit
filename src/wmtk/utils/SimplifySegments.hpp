#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <cstddef>

namespace wmtk::utils {

/**
 * @brief Greedy shortest-segment simplification of a 2D segment network, constrained to an
 * envelope.
 *
 * The 1D counterpart of the input simplification tetwild runs before insertion (see
 * components/shortest_edge_collapse). It exists for the same reason: every input vertex
 * becomes a constrained vertex of the exact arrangement and then a surface vertex of the
 * initial mesh, and inputs are routinely far finer than the target edge length. Coarsening
 * them here, in 1D, is much cheaper than letting the 2D optimizer do it with rational
 * positions and per-triangle envelope queries -- and it shrinks the arrangement itself.
 *
 * Repeatedly collapses the shortest segment whose removal keeps every affected segment
 * inside `envelope`:
 *
 * - Vertices of valence != 2 are frozen: open endpoints, T/Y junctions, and (because they
 *   have valence >= 4) any vertex shared between two different input curves. A segment with
 *   one frozen endpoint still collapses, onto the frozen vertex's exact position, so the
 *   junction geometry is preserved bit-exactly; only both-endpoints-frozen is rejected.
 *   This is where it deviates from the 3D version, which rejects outright -- on a curve
 *   network that rule leaves a ring of un-collapsible short segments around every junction.
 * - Otherwise the two endpoints merge at their midpoint, as in 3D.
 * - Collapses that would leave a degenerate or duplicated segment are rejected.
 *
 * Serial and deterministic: the queue is keyed on (squared length, segment id), so equal
 * lengths never leave the order to chance.
 *
 * @param[in,out] V  Nx2 vertex positions; compacted in place
 * @param[in,out] E  Mx2 segment endpoint indices; compacted in place
 * @param envelope   must already be initialised around the *input* (V, E) with the desired
 *                   thickness -- the caller owns that choice
 * @return the number of vertices removed
 */
size_t simplify_segments(MatrixXd& V, MatrixXi& E, const SampleEnvelope& envelope);

} // namespace wmtk::utils
