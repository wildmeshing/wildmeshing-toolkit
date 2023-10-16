#pragma once
#include <wmtk/function/utils/autodiff.h>
#include "DifferentiableFunction.hpp"
namespace wmtk::function {

class AutodiffFunction : public DifferentiableFunction
{
public:
    using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
    using Scalar = typename DScalar::Scalar;
    static_assert(
        std::is_same_v<Scalar, double>); // MTAO: i'm leaving scalar here but is it ever not double?
    AutodiffFunction(const Mesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle);

    virtual ~AutodiffFunction();

public:
    double get_value(const Tuple& tuple) const override;
    Eigen::VectorXd get_gradient(const Tuple& tuple) const override;
    Eigen::MatrixXd get_hessian(const Tuple& tuple) const override;

protected:
    virtual DScalar get_value_autodiff(const Tuple& tuple) const = 0;
};
} // namespace wmtk::function
