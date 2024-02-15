#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
/**
 * @brief The closed star contains the input simplex, all its top dimension cofaces, and their
 * faces.
 *
 * @param mesh The mesh to which the simplex and its closed star belong.
 * @param simplex The simplex of which the closed star is computed.
 * @return A SimplexCollection with the closed star.
 */
SimplexCollection
closed_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);
} // namespace wmtk::simplex