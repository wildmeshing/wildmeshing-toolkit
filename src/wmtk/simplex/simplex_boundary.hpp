#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
    SimplexCollection simplex_boundary(const Mesh& mesh, const Simplex& simplex);
}