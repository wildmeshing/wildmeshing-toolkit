#include "InteriorVertexInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {
InteriorVertexInvariant::InteriorVertexInvariant(const Mesh& m)
    : InteriorSimplexInvariant(m, PrimitiveType::Vertex)
{}
} // namespace wmtk::invariants
