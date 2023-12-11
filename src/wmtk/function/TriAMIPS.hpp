#pragma once
#include "AutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class TriAMIPS : public TriangleAutodiffFunction
{
public:
    TriAMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    ~TriAMIPS();

protected:
    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;
};

} // namespace wmtk::function
