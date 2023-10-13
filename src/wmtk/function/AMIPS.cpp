#include "AMIPS.hpp"

namespace wmtk::function {
AMIPS::AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AutodiffFunction(mesh, vertex_attribute_handle)
{}

} // namespace wmtk::function
