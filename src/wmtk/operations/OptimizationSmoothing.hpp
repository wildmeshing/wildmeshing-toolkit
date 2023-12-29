#pragma once

#include <wmtk/function/Function.hpp>
#include "AttributesUpdateBase.hpp"

#include <polysolve/nonlinear/Problem.hpp>


namespace polysolve::nonlinear {
class Solver;
}

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {

class OptimizationSmoothing : public AttributesUpdateBase
{
private:
    class WMTKProblem : public polysolve::nonlinear::Problem
    {
    public:
        using typename polysolve::nonlinear::Problem::Scalar;
        using typename polysolve::nonlinear::Problem::THessian;
        using typename polysolve::nonlinear::Problem::TVector;

        WMTKProblem(
            Mesh& mesh,
            const MeshAttributeHandle<double>& handle,
            const simplex::Simplex& simplex,
            InvariantCollection& invariants,
            const wmtk::function::Function& energy);

        TVector initial_value() const;

        double value(const TVector& x) override;
        void gradient(const TVector& x, TVector& gradv) override;
        void hessian(const TVector& x, THessian& hessian) override
        {
            throw std::runtime_error("Sparse functions do not exist, use dense solver");
        }
        void hessian(const TVector& x, Eigen::MatrixXd& hessian) override;

        void solution_changed(const TVector& new_x) override;

        bool is_step_valid(const TVector& x0, const TVector& x1) const override;

    private:
        MeshAttributeHandle<double> m_handle;
        Accessor<double> m_accessor;
        const simplex::Simplex& m_simplex;
        const wmtk::function::Function& m_energy;

        InvariantCollection& m_invariants;
    };

public:
    OptimizationSmoothing(std::shared_ptr<wmtk::function::Function> energy);

    std::vector<Simplex> execute(const Simplex& simplex) override;

private:
    std::shared_ptr<wmtk::function::Function> m_energy;
    std::shared_ptr<polysolve::nonlinear::Solver> m_solver;
};

} // namespace wmtk::operations
