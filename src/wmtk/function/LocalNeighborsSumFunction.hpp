#pragma once

#include <wmtk/utils/Logger.hpp>
#include "Function.hpp"
namespace wmtk::function {

class PerSimplexFunction;

class LocalNeighborsSumFunction : public Function
{
public:
    static int grad_cnt, hess_cnt;
    LocalNeighborsSumFunction(
        Mesh& mesh,
        const attribute::MeshAttributeHandle& handle,
        PerSimplexFunction& function);
    /**
     * @brief collects the local neigh and call the same m_function on all simplicies
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt this argument.
     * @return double The value of the function at the input simplex.
     */
    double get_value(const simplex::Simplex& variable_simplex) const override;

    /**
     * @brief get_gradient collects the local neigh and call the gradient of m_function on all
     * simplicies
     *
     * This variable is represented by the input argument variable_simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt the attributes of this
     * argument.
     * @return Eigen::VectorXd The gradient of the function. The dimension of the vector is equal to
     * the dimension of the embedded space.
     */
    Eigen::VectorXd get_gradient(const simplex::Simplex& variable_simplex) const override;

    // TODO: should differentiable function be required to be twice differentiable?
    Eigen::MatrixXd get_hessian(const simplex::Simplex& variable_simplex) const override;

    std::vector<simplex::Simplex> domain(const simplex::Simplex& variable_simplex) const override;
    void print_cnt() const override
    {
        logger().debug("LocalNeighborsSumFunction:: grad cnt {}, hess cnt {}", grad_cnt, hess_cnt);
    }

private:
    PerSimplexFunction& m_function;
    PrimitiveType m_domain_simplex_type;
};
} // namespace wmtk::function
