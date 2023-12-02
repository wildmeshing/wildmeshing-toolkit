#pragma once
#include "AutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class AMIPS : public TriangleAutodiffFunction
{
public:
    AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    ~AMIPS();

protected:
    DScalar eval(
        const Simplex& domain_simplex,
        DSVec& coordinate0,
        DSVec& coordinate1,
        DSVec& coordinate2) const override;
};

} // namespace wmtk::function
