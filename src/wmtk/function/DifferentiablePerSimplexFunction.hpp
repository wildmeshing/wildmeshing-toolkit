#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/attribute/MeshAttributes.hpp>
#include "PerSimplexFunction.hpp"
namespace wmtk {
namespace function {
class DifferentiablePerSimplexFunction : public PerSimplexFunction
{
public:
    /**
     * @brief Construct a new DifferentiablePerSimplexFunction object where the function is defined
     * over simplices of simplex_type. And the differentiation is taken wrt the
     * attribute_handle.primitive_type()
     *
     * @param mesh
     * @param simplex_type
     */
    DifferentiablePerSimplexFunction(
        const Mesh& mesh,
        const PrimitiveType& simplex_type,
        const attribute::MeshAttributeHandle<double>& variable_attribute_handle);
    virtual ~DifferentiablePerSimplexFunction();

public:
    virtual Eigen::VectorXd get_gradient(const Simplex& s) const = 0;
    virtual Eigen::MatrixXd get_hessian(const Simplex& s) const = 0;

    const attribute::MeshAttributeHandle<double>& get_variable_attribute_handle() const;

    /**
     * @brief the function should be defined on the simplex of type A and the differntiation is
     * taken wrt simplex type B. The definition mandate (1) A should be coface with B, (2) the type
     * of the attribute_handle should be of the same type as B.
     *
     * @param s_type simplex Type B as defined above
     */
    void assert_function_type(const PrimitiveType& s_type) const;
    long embedded_dimension() const;

private:
    const attribute::MeshAttributeHandle<double> m_attribute_handle;
};
} // namespace function
} // namespace wmtk