#pragma once

#include "iterable/LinkIterable.hpp"

namespace wmtk::simplex {
LinkIterable link_iterable(const Mesh& mesh, const Simplex& simplex);
}