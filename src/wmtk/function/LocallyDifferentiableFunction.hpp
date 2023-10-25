#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include "DifferentiablePerSimplexFunction.hpp"
#include "Function.hpp"
namespace wmtk {
namespace function {
class LocallyDifferentiableFunction : public Function
{
public:
    LocallyDifferentiableFunction(
        std::shared_ptr<DifferentiablePerSimplexFunction>&& function,
        const PrimitiveType& my_simplex_type);
    virtual ~LocallyDifferentiableFunction();

public:
    Eigen::VectorXd get_local_gradient(const Simplex& my_simplex) const;
    Eigen::MatrixXd get_local_hessian(const Simplex& my_simplex) const;
    Eigen::VectorXd get_local_gradient(const Tuple& my_tuple) const;
    Eigen::MatrixXd get_local_hessian(const Tuple& my_tuple) const;

    double get_value_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::VectorXd get_gradient_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::MatrixXd get_hessian_sum(const std::vector<Simplex>& coface_simplices) const;

private:
    std::shared_ptr<DifferentiablePerSimplexFunction> m_function;
    const PrimitiveType m_my_simplex_type;
};
} // namespace function
} // namespace wmtk