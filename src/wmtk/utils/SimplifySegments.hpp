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
 * - Open endpoints (valence < 2) are always frozen, whatever `use_link_condition` says.
 *   That mirrors the 3D ShortestEdgeCollapse, whose freeze_boundary() runs unconditionally:
 *   the envelope is one-sided (output inside envelope(input)) and so cannot see a curve
 *   eroding inwards from its own tip.
 * - Junctions (valence > 2) are frozen only when `use_link_condition` is set -- T/Y
 *   junctions and, because they have valence >= 4, any vertex shared between two different
 *   input curves.
 * - A segment with one frozen endpoint still collapses, onto the frozen vertex's exact
 *   position, so the junction geometry is preserved bit-exactly; only both-endpoints-frozen
 *   is rejected, because merging two frozen vertices has to move one of them. Same rule as
 *   the 3D ShortestEdgeCollapse. Rejecting whenever *either* endpoint is frozen would
 *   strand one un-collapsible vertex on every chain between two frozen ones -- twice the
 *   segments on a network of short chains.
 * - Otherwise the two endpoints merge at their midpoint, as in 3D.
 * - Segments that become degenerate or duplicated abort the collapse when
 *   `use_link_condition` is set, and are simply dropped when it is not -- the 2D
 *   counterpart of 3D collapses that would create a non-manifold edge or vertex.
 *
 * Serial and deterministic: the queue is keyed on (squared length, segment id), so equal
 * lengths never leave the order to chance.
 *
 * @param[in,out] V  Nx2 vertex positions; compacted in place
 * @param[in,out] E  Mx2 segment endpoint indices; compacted in place
 * @param envelope   must already be initialised around the *input* (V, E) with the desired
 *                   thickness -- the caller owns that choice
 * @param use_link_condition  freeze junctions and veto degenerate/duplicated results, so the
 *                   simplification cannot change the topology of the curve network. When
 *                   false the network simplifies much further on dirty inputs, at the cost
 *                   of merging junctions and separate curves that pass within the envelope.
 * @return the number of vertices removed
 */
size_t simplify_segments(
    MatrixXd& V,
    MatrixXi& E,
    const SampleEnvelope& envelope,
    bool use_link_condition = true);

} // namespace wmtk::utils
