
#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
namespace wmtk::operations::tri_mesh::internal {
class SeamlessGradientDescent : public VertexSmoothUsingDifferentiableEnergy
{
public:
    SeamlessGradientDescent(
        TriMesh& seamed_mesh,
        std::shared_ptr<TriMesh> cut_mesh,
        MeshAttributeHandle<double> uv_handle,
        const Simplex& v_on_seamed_mesh,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

    // std::vector<double> priority() const override;

protected:
    Eigen::VectorXd get_descent_direction(function::utils::DifferentiableFunctionEvaluator&) const;
    bool execute() override;
    std::string name() const override;

    double evaluate_energy() const;

    bool execute_on_interior(const Simplex& v_on_cut_mesh);
    bool execute_on_boundary(const std::vector<Simplex>& vs_on_cut_mesh);

private:
    std::shared_ptr<TriMesh> m_cut_mesh;
    MeshAttributeHandle<double> m_uv_handle;
};
} // namespace wmtk::operations::tri_mesh::internal