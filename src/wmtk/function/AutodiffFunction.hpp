#pragma once
#include "DifferentiableFunction.hpp"
namespace wmtk {
namespace function {

using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
using Scalar = typename DScalar::Scalar;

class AutodiffFunction : public DifferentiableFunction
{
public:
    AutodiffFunction(const Mesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    virtual ~AutodiffFunction() = default;

public:
    double get_value(const Tuple& tuple) const override;
    Eigen::VectorXd get_gradient(const Tuple& tuple) const override;
    Eigen::MatrixXd get_hessian(const Tuple& tuple) const override;

protected:
    virtual DScalar get_value_autodiff(const Tuple& tuple) const = 0;
};
} // namespace function
} // namespace wmtk