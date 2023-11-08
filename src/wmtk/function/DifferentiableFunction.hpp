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
     * @brief evaluate the gradient of the funciton f(x) defined wrt the variable x. The vairable is
     * represented by a simplex which is of the same type as the coordinate_attribute
     *
     * @param simplex
     * @return Eigen::VectorXd
     */
    virtual Eigen::VectorXd get_gradient(const simplex::Simplex& variable_simplex) const = 0;

    // TODO: should differentiable function be required to be twice differentiable?
    virtual Eigen::MatrixXd get_hessian(const simplex::Simplex& variable_simplex) const = 0;

    /**
     * @brief return the embedded space dimension (e.g. 2 for a 2D space, 3 for a 3D space)
     * (is not directly related to the type of the coordinate_attribute)
     *
     * @return long
     */
    long embedded_dimension() const;
    virtual MeshAttributeHandle<double> get_coordinate_attribute_handle() const = 0;
};
} // namespace wmtk::function
