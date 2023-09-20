#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {

SimplexCollection coface_cells(const Mesh& mesh, const Simplex& simplex);

}