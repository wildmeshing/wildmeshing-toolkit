#pragma once
#include <wmtk/function/utils/autodiff.h>
#include "DifferentiablePerSimplexFunction.hpp"
namespace wmtk::function {

class AutodiffFunction : public DifferentiablePerSimplexFunction
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
    double get_value(const Simplex& simplex) const override;
    Eigen::VectorXd get_gradient(const Simplex& simplex) const override;
    Eigen::MatrixXd get_hessian(const Simplex& simplex) const override;

protected:
    virtual DScalar get_value_autodiff(const Simplex& simplex) const = 0;
};
} // namespace wmtk::function
