#pragma once
#include "PerSimplexDifferentiableAutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is the implementation of the AMIPS energy function of a triangle mesh that can be
 * embedded in 2d or 3d. It uses autodiff encoding for differentiations.
 *
 */
class TriangleAMIPS : public TriangleAutodiffFunction
{
public:
    TriangleAMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    ~TriangleAMIPS();

protected:
    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;
};

} // namespace wmtk::function
