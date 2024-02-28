#pragma once

#include <wmtk/attribute//MeshAttributeHandle.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <Eigen/Core>

namespace wmtk::function {

class Function
{
public:
    Function(Mesh& mesh, const attribute::MeshAttributeHandle& handle);
    virtual ~Function() {}

    /**
     * @brief Given a function f(x), get_value evaluate the function at the input simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt this argument.
     * @return double The value of the function at the input simplex.
     */
    virtual double get_value(const simplex::Simplex& variable_simplex) const = 0;

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

    /**
     * @brief get_hessian evaluates the hessian of the function f(x) defined wrt the variable x.
     *
     * This variable is represented by the input argument variable_simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt the attributes of this
     * argument.
     * @return Eigen::MatrixXd The hessian of the function. The dimension of the matrix is equal to
     * the dimension of the embedded space.
     */
    virtual Eigen::MatrixXd get_hessian(const simplex::Simplex& variable_simplex) const = 0;

    inline Mesh& mesh() { return m_mesh; }
    inline const Mesh& mesh() const { return m_mesh; }
    inline PrimitiveType attribute_type() const { return m_handle.primitive_type(); };
    int64_t embedded_dimension() const;
    inline const attribute::MeshAttributeHandle& attribute_handle() const { return m_handle; }
    virtual std::vector<simplex::Simplex> domain(
        const simplex::Simplex& variable_simplex) const = 0;


private:
    Mesh& m_mesh;
    attribute::MeshAttributeHandle m_handle;
};
} // namespace wmtk::function
