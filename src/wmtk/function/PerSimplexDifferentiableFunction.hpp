#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/attribute/MeshAttributes.hpp>
#include "DifferentiableFunction.hpp"
#include "PerSimplexFunction.hpp"
namespace wmtk {
namespace function {
class PerSimplexDifferentiableFunction : public PerSimplexFunction
{
public:
    /**
     * @brief Construct a new PerSimplexDifferentiableFunction object where the function is defined
     * over domains of domain_simplex_type (for example, triangle or tet). If we write the function
     * as f(x), where the variable x is the attribute that the function is defined with respect to.
     * And this attribute is accessed through the attribute_handle.
     *
     * @param mesh
     * @param simplex_type The type of the domain that the function is defined over
     * @param attribute_handle The handle to the attribute that differentiation is with respect to
     */
    PerSimplexDifferentiableFunction(
        const Mesh& mesh,
        PrimitiveType domain_simplex_type,
        const attribute::MeshAttributeHandle<double>& attribute_handle);
    ~PerSimplexDifferentiableFunction();

public:
    virtual Eigen::VectorXd get_gradient(
        const Simplex& domain_simplex,
        const Simplex& variable_simplex) const = 0;
    virtual Eigen::MatrixXd get_hessian(
        const Simplex& domain_simplex,
        const Simplex& variable_simplex) const = 0;

    long embedded_dimension() const;
    attribute::MeshAttributeHandle<double> get_coordinate_attribute_handle() const;
    PrimitiveType get_coordinate_attribute_primitive_type() const;

    // computes the sum over a set of simplices - assumes each simplex has the same dimension as the
    // function's simplex type
    Eigen::VectorXd get_gradient_sum(
        const std::vector<Simplex>& simplices,
        const Simplex& variable_simplex) const;
    Eigen::MatrixXd get_hessian_sum(
        const std::vector<Simplex>& simplices,
        const Simplex& variable_simplex) const;

    Eigen::VectorXd get_gradient_sum(
        const std::vector<Tuple>& simplices,
        const Tuple& variable_simplex) const;
    Eigen::MatrixXd get_hessian_sum(
        const std::vector<Tuple>& simplices,
        const Tuple& variable_simplex) const;

protected:
    Simplex as_coordinate_attribute_simplex(const Tuple& t) const;

private:
    const MeshAttributeHandle<double> m_coordinate_attribute_handle;
};
} // namespace function
} // namespace wmtk
