#pragma once

#include <polysolve/Types.hpp>
#include <wmtk/function/Function.hpp>
#include "AttributesUpdate.hpp"


namespace polysolve::nonlinear {
class Solver;
}

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {

class SeamlessSmoothing : public AttributesUpdate
{
private:
    class WMTKProblem;

public:
    SeamlessSmoothing(std::shared_ptr<wmtk::function::Function> energy);

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
    std::shared_ptr<wmtk::function::Function> m_energy;
    std::shared_ptr<polysolve::nonlinear::Solver> m_solver;


    polysolve::json m_linear_solver_params;
    polysolve::json m_nonlinear_solver_params;

    void create_solver();
};

} // namespace wmtk::operations
