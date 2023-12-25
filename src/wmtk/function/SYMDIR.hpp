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
        const TriMesh& ref_mesh,
        const TriMesh& uv_mesh,
        const MeshAttributeHandle<double>& ref_attribute_handle,
        const MeshAttributeHandle<double>& uv_attribute_handle,
        bool do_integral = false);

    ~SYMDIR();

    DScalar::Scalar get_energy_avg() const;
    DScalar::Scalar get_energy_max() const;

protected:
    DScalar eval(const Simplex& domain_simplex, const std::array<DSVec, 3>& coordinates)
        const override;
    const TriMesh& m_ref_mesh;
    std::optional<const MeshAttributeHandle<double>> m_vertex_attribute_handle_opt;
    bool m_uniform_reference = true;
    bool m_do_integral;
};

} // namespace wmtk::function
