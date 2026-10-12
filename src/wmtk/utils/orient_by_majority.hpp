#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace wmtk::utils {

/// Two oriented elements sharing a manifold junction (a facet edge in 3D, a curve vertex in
/// 2D): `consistent` if, as they stand, they induce opposite orientations on it.
struct OrientationLink
{
    uint32_t a;
    uint32_t b;
    bool consistent;
};

struct OrientationRepair
{
    /// Per element: must it be reversed.
    std::vector<bool> turn;
    size_t n_turned = 0;
    /// Linked components no orientation makes consistent (a Moebius strip, or links that
    /// contradict each other some other way); they are left as they were.
    size_t n_non_orientable = 0;
};

/**
 * @brief Orient linked elements consistently, component by component, each component keeping
 * the orientation that most of its weight already has.
 *
 * The components are those of the graph `links` makes on the n elements. On a component whose
 * links are all consistent nothing turns; elsewhere the minority side -- by total weight -- is
 * reversed. Deterministic: a tie keeps the side the component's lowest element is on.
 *
 * O((n + #links) alpha(n)).
 */
OrientationRepair orient_by_majority(
    size_t n,
    const std::vector<OrientationLink>& links,
    const std::vector<double>& weight);

} // namespace wmtk::utils
