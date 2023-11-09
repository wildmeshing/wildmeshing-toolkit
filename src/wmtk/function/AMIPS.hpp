#pragma once
#include "AutodiffFunction.hpp"
namespace wmtk::function {
class AMIPS : public AutodiffFunction
{
public:
    AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);
};

} // namespace wmtk::function
