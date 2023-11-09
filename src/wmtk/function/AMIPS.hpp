#pragma once
#include "AutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace wmtk::function {
class AMIPS : public TriangleAutodiffFunction
{
public:
    AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    ~AMIPS();

protected:
    DScalar eval(DSVec& coordinate0, DSVec& coordinate1, DSVec& coordinate2) const override;
};

} // namespace wmtk::function
