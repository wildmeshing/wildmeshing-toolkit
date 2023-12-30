#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {
SimplexCollection k_ring(const Mesh& mesh, const Simplex& simplex, long k);
}
