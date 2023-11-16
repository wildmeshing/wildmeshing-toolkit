#pragma once
#include <Eigen/Core>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include "Function.hpp"
namespace wmtk::function {

class DifferentiableFunction : public virtual Function
{
public:
    /**
     * @brief get_gradient evaluates the gradient of the function f(x) defined wrt the variable x.
     *
     * This variable is represented by the input argument variable_simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt the attributes of this
     * argument.
     * @return Eigen::VectorXd The gradient of the function. The dimension of the vector is equal to
     * the dimension of the embedded space.
     */
    virtual Eigen::VectorXd get_gradient(const simplex::Simplex& variable_simplex) const = 0;

    // TODO: should differentiable function be required to be twice differentiable?
    virtual Eigen::MatrixXd get_hessian(const simplex::Simplex& variable_simplex) const = 0;

    /**
     * @brief return the embedded space dimension (for example 2 for a 2D space, 3 for a 3D space)
     * (is not directly related to the type of the coordinate_attribute)
     *
     * @return long
     */
    long embedded_dimension() const;
    virtual MeshAttributeHandle<double> get_coordinate_attribute_handle() const = 0;
};
} // namespace wmtk::function
