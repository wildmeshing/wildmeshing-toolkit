#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
// returns all simplices that lie on the boundary of the input simplex (i.e all cofaces)
// This does not include itself
SimplexCollection all_faces(const Mesh& mesh, const std::vector<Simplex>& simplices);
} // namespace wmtk::simplex
