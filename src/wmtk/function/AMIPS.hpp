#pragma once
#include <wmtk/image/Image.hpp>
#include "AutodiffFunction.hpp"
#include "utils/AutoDiffUtils.hpp"
#include "utils/DofsToPosition.hpp"
namespace wmtk::function {
class AMIPS : public AutodiffFunction
{
public:
    AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);
};

} // namespace wmtk::function
