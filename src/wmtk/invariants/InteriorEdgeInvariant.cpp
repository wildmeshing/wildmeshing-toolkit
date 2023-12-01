#include "InteriorEdgeInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
InteriorEdgeInvariant::InteriorEdgeInvariant(const Mesh& m)
    : InteriorSimplexInvariant(m, PrimitiveType::Edge)
{}
} // namespace wmtk::invariants
