#pragma once
#include "AutodiffFunction.hpp"
#include "TriangleAutodiffFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is the implementation of the Symmetric Dirichlet energy function of a triangle mesh.
 * It uses autodiff encoding for differentiations.
 *
 */
class SYMDIR : public TriangleAutodiffFunction
{
    using DScalar = typename AutodiffFunction::DScalar;

public:
    SYMDIR(const TriMesh& uv_mesh, const MeshAttributeHandle<double>& uv_attribute_handle);

    SYMDIR(
        const TriMesh& mesh,
        const TriMesh& uv_mesh,
        const MeshAttributeHandle<double>& vertex_attribute_handle,
        const MeshAttributeHandle<double>& uv_attribute_handle);

    ~SYMDIR();

protected:
    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;

    Eigen::Vector3<DScalar> A, B, C; // reference triangle coordinates
};

} // namespace wmtk::function
