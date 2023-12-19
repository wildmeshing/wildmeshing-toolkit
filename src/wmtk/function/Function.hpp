#pragma once

#include <polysolve/nonlinear/Problem.hpp>

#include <wmtk/Accessor.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::function {

class Function : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    Function(Mesh& mesh, MeshAttributeHandle<double>& handle, const simplex::Simplex& simplex);


    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        throw std::runtime_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, Eigen::MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;

    bool is_step_valid(const TVector& x0, const TVector& x1) const override;


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
    long embedded_dimension() const;


private:
    Mesh& m_mesh;
    MeshAttributeHandle<double> m_handle;
    Accessor<double> m_accessor;
    const simplex::Simplex& m_simplex;
};
} // namespace wmtk::function
