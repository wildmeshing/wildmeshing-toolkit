#pragma once
#include <wmtk/function/utils/autodiff.h>
#include "PerSimplexDifferentiableFunction.hpp"
namespace wmtk::function {

class AutodiffFunction : public PerSimplexDifferentiableFunction
{
public:
    using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
    using Scalar = typename DScalar::Scalar;
    static_assert(
        std::is_same_v<Scalar, double>); // MTAO: i'm leaving scalar here but is it ever not double?
    AutodiffFunction(
        const Mesh& mesh,
        const PrimitiveType& simplex_type,
        const attribute::MeshAttributeHandle<double>& variable_attribute_handle);

    virtual ~AutodiffFunction();

public:
    using PerSimplexFunction::get_value;
    using PerSimplexDifferentiableFunction::get_hessian;
    using PerSimplexDifferentiableFunction::get_gradient;
    double get_value(const Tuple& tuple) const final override;
    Eigen::VectorXd get_gradient(const Tuple& tuple) const final override;
    Eigen::MatrixXd get_hessian(const Tuple& tuple) const final override;


protected:
    virtual DScalar get_value_autodiff(const Tuple& simplex) const = 0;
    DScalar get_value_autodiff(const Simplex& simplex) const;
};
} // namespace wmtk::function
