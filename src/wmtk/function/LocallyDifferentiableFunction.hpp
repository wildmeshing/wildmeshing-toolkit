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
    LocallyDifferentiableFunction(std::unique_ptr<DifferentiablePerSimplexFunction>&& function);
    virtual ~LocallyDifferentiableFunction();

public:
    Eigen::VectorXd get_one_ring_gradient(
        const Simplex& my_simplex,
        const PrimitiveType& cofaces_type) const;
    Eigen::MatrixXd get_one_ring_hessian(
        const Simplex& my_simplex,
        const PrimitiveType& cofaces_type) const;

    double get_value_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::VectorXd get_gradient_sum(const std::vector<Simplex>& coface_simplices) const;
    Eigen::MatrixXd get_hessian_sum(const std::vector<Simplex>& coface_simplices) const;

private:
    std::unique_ptr<DifferentiablePerSimplexFunction> m_function;
};
} // namespace function
} // namespace wmtk