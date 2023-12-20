#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/operations/Operation.hpp>
#include "AttributesUpdateBase.hpp"

#include <polysolve/nonlinear/Problem.hpp>

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {
class OptSmoothing;

template <>
struct OperationSettings<OptSmoothing> : public OperationSettings<AttributesUpdateBase>
{
    OperationSettings<OptSmoothing>(Mesh& m)
        : OperationSettings<AttributesUpdateBase>(m)
    {}

    std::unique_ptr<wmtk::function::Function> energy;
    // coordinate for teh attribute used to evaluate the energy
    MeshAttributeHandle<double> coordinate_handle;

    void create_invariants();
};

class OptSmoothing : public AttributesUpdateBase
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
            MeshAttributeHandle<double>& handle,
            const simplex::Simplex& simplex,
            std::unique_ptr<wmtk::function::Function>& energy);


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
        Mesh& m_mesh;
        MeshAttributeHandle<double> m_handle;
        Accessor<double> m_accessor;
        const simplex::Simplex& m_simplex;
        std::unique_ptr<wmtk::function::Function>& m_energy;
    };

public:
    OptSmoothing(Mesh& m, const Simplex& t, const OperationSettings<OptSmoothing>& settings);

    std::string name() const override;
    bool execute() override;

protected:
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<OptSmoothing>& m_settings;
};

} // namespace wmtk::operations
