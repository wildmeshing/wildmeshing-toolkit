#pragma once
#include <Eigen/Core>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include "Function.hpp"
namespace wmtk::function {

class DifferentiableFunction : public virtual Function
{
public:
    // evaluates the gradient of the tuple

    virtual Eigen::VectorXd get_gradient(const simplex::Simplex& tuple) const = 0;

    // TODO: should differentiable function be required to be twice differentiable?
    virtual Eigen::MatrixXd get_hessian(const simplex::Simplex& tuple) const = 0;

    long embedded_dimension() const;
    virtual MeshAttributeHandle<double> get_coordinate_attribute_handle() const = 0;

};
} // namespace wmtk::function
