#pragma once

#include <polysolve/Types.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include "AttributesUpdate.hpp"


namespace polysolve::nonlinear {
class Solver;
}

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {

class AMIPSOptimizationSmoothingPeriodic : public AttributesUpdate
{
private:
    template <int S>
    class WMTKAMIPSProblem;

public:
    AMIPSOptimizationSmoothingPeriodic(
        Mesh& periodic_mesh,
        Mesh& position_mesh,
        const attribute::MeshAttributeHandle& coords);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

    const polysolve::json& linear_solver_params() const { return m_linear_solver_params; }
    const polysolve::json& nonlinear_solver_params() const { return m_nonlinear_solver_params; }


    void set_linear_solver_params(const polysolve::json& params)
    {
        m_linear_solver_params = params;
        create_solver();
    }

    void set_nonlinear_solver_params(const polysolve::json& params)
    {
        m_nonlinear_solver_params = params;
        create_solver();
    }

private:
    std::shared_ptr<polysolve::nonlinear::Solver> m_solver;
    Mesh& m_periodic_mesh;
    Mesh& m_position_mesh;
    const attribute::MeshAttributeHandle& m_coordinate_handle;

    polysolve::json m_linear_solver_params;
    polysolve::json m_nonlinear_solver_params;

    function::AMIPS m_amips;

    void create_solver();
};

} // namespace wmtk::operations
