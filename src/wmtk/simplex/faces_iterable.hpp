#pragma once

#include "iterable/FacesIterable.hpp"

namespace wmtk::simplex {
FacesIterable faces_iterable(const Mesh& mesh, const Simplex& simplex);
}
