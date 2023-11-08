#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include "DifferentiableFunction.hpp"
#include "LocalFunction.hpp"
namespace wmtk::function {
class PerSimplexDifferentiableFunction;
;
class LocalDifferentiableFunction : public LocalFunction, public DifferentiableFunction
{
public:
    LocalDifferentiableFunction(std::shared_ptr<PerSimplexDifferentiableFunction> function);
    ~LocalDifferentiableFunction();

public:
    Eigen::VectorXd get_gradient(const Simplex& variable_simplex) const override;
    Eigen::MatrixXd get_hessian(const Simplex& variable_simplex) const override;

    const PerSimplexDifferentiableFunction& per_simplex_function() const;
    std::shared_ptr<PerSimplexDifferentiableFunction> per_simplex_function_ptr() const;

    attribute::MeshAttributeHandle<double> get_coordinate_attribute_handle() const final override;
};
} // namespace wmtk::function
