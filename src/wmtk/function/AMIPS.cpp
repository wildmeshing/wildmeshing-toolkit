#include "AMIPS.hpp"
#include <wmtk/TriMesh.hpp>

namespace wmtk::function {
AMIPS::AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AutodiffFunction(mesh, PrimitiveType::Face, vertex_attribute_handle)
{}

} // namespace wmtk::function
