#pragma once
#include <Eigen/Core>
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/function/DifferentiableFunction.hpp>
#include "FunctionEvaluator.hpp"

namespace wmtk::function::utils {


// Evaluates a function at a particular vertex of a mesh
// NOTE that this modifies attributes in the mesh.
// This should only be called from within a Scope so evaluates can be undone
//
class DifferentiableFunctionEvaluator : public FunctionEvaluator
{
public:
    DifferentiableFunctionEvaluator(
        const function::DifferentiableFunction& function,
        Accessor<double>& accessor,
        const Simplex& simplex);
    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    Vector get_gradient() const;
    Matrix get_hessian() const;


    template <typename Derived>
    Vector get_gradient(const Eigen::MatrixBase<Derived>& v);
    template <typename Derived>
    Matrix get_hessian(const Eigen::MatrixBase<Derived>& v);

    Vector get_gradient(double v);
    Matrix get_hessian(double v);
    const function::DifferentiableFunction& function() const;

    const std::vector<Tuple>& cofaces_single_dimension() const;

private:
    // cache the top simplices
    const function::DifferentiableFunction& m_function;
    // std::vector<Tuple> m_cofaces_single_dimension;
    // std::vector<Tuple> compute_cofaces_single_dimension() const;
    // std::vector<Tuple> compute_top_dimension_cofaces() const;
};

template <typename Derived>
auto DifferentiableFunctionEvaluator::get_gradient(const Eigen::MatrixBase<Derived>& v) -> Vector
{
    store(v);
    return get_gradient();
}
template <typename Derived>
auto DifferentiableFunctionEvaluator::get_hessian(const Eigen::MatrixBase<Derived>& v) -> Matrix
{
    store(v);
    return get_hessian();
}
} // namespace wmtk::function::utils
