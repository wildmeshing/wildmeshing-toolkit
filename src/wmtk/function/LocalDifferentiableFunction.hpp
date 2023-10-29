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
    LocalDifferentiableFunction(
        std::shared_ptr<PerSimplexDifferentiableFunction> function,
        const PrimitiveType& simplex_type);
    virtual ~LocalDifferentiableFunction();

public:
    //Eigen::VectorXd get_local_gradient(const Simplex& simplex) const override;
    //Eigen::MatrixXd get_local_hessian(const Simplex& simplex) const override;
    //Eigen::VectorXd get_local_gradient(const Tuple& tuple) const;
    //Eigen::MatrixXd get_local_hessian(const Tuple& tuple) const;

    Eigen::VectorXd get_gradient(const Simplex& simplex) const override;
    Eigen::MatrixXd get_hessian(const Simplex& simplex) const override;
    Eigen::VectorXd get_gradient(const Tuple& tuple) const;
    Eigen::MatrixXd get_hessian(const Tuple& tuple) const;

    const PerSimplexDifferentiableFunction& per_simplex_function() const;
    std::shared_ptr<PerSimplexDifferentiableFunction> per_simplex_function_ptr() const;

    attribute::MeshAttributeHandle<double> get_coordinate_attribute_handle() const final override;

private:
    const PrimitiveType m_simplex_type;
};
} // namespace wmtk::function
