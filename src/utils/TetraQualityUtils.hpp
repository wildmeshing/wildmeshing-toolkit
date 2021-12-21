#pragma once

#include <Eigen/Core>

namespace wmtk {

/**
 * Newton method with regard to AMIPS_energy.
 *
 * @param stack with flattened 4x3 vertices positions, with the mover vertex always on the front.
 * This is the same convention with AMIPS_energy
 * @return Descend direction as computed from Newton method, (or gradient descent if Newton fails).
 */
Eigen::Vector3d newton_direction_from_stack(std::vector<std::array<double, 12>>& stack);

/**
 * Reorders indices in a tetrahedron such that v0 is on the front. Using the tetra symmetry to
 * preserve orientation. Assumes v0 in tetra.
 * @param conn
 * @param v0
 * @return std::array<size_t, 4>
 * TODO: this can be easily templated.
 */
std::array<size_t, 4> orient_preserve_tet_reorder(const std::array<size_t, 4>& tetra, size_t v0);
} // namespace wmtk