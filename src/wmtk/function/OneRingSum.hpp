#pragma once
#include <wmtk/image/Image.hpp>
#include "AutodiffFunction.hpp"
#include "utils/DifferentiableFunction.hpp"
#include "utils/DofsToPosition.hpp"

namespace wmtk {
namespace function {
class OneRingSum : public DifferentiableFunction
{
public:
    OneRingSum(
        const TriMesh& mesh,
        const MeshAttributeHandle<double>& vertex_attribute_handle,
        const std::unique_ptr<DifferentiableFunction>& func);

    double get_value(const Tuple& tuple) const override;
    Eigen::VectorXd get_gradient(const Tuple& tuple) const override;
    Eigen::MatrixXd get_hessian(const Tuple& tuple) const override;

private:
    const std::unique_ptr<DifferentiableFunction>& m_func;
};
} // namespace function
} // namespace wmtk