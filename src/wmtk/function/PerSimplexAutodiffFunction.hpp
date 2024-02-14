#pragma once

#include "PerSimplexFunction.hpp"

#include <wmtk/function/utils/autodiff.h>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

#include <optional>

namespace wmtk::function {
/**
 * @brief This is an extension of the PerSimplexFunction class that uses autodiff
 * encoding for differentiations
 *
 */
class PerSimplexAutodiffFunction : public PerSimplexFunction
{
public:
    using DScalar = DScalar2<
        double,
        Eigen::Matrix<double, -1, 1, 0, 3, 1>,
        Eigen::Matrix<double, -1, -1, 0, 3, 3>>;
    using Scalar = typename DScalar::Scalar;
    using DSVec = Eigen::Matrix<DScalar, -1, 1, 0, 3, 1>;


    static_assert(
        std::is_same_v<Scalar, double>); // MTAO: i'm leaving scalar here but is it ever not double?
    PerSimplexAutodiffFunction(
        const Mesh& mesh,
        const PrimitiveType primitive_type,
        const attribute::MeshAttributeHandle& variable_attribute_handle);

    ~PerSimplexAutodiffFunction();


    double get_value(const simplex::Simplex& domain_simplex) const override;

    Eigen::VectorXd get_gradient(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;

    Eigen::MatrixXd get_hessian(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;

protected:
    /**
     * @brief This is a helper function that obtains the coordinates of the variables for the
     * function f(x) where f is defined over a domain consists of the input argument domain_tuples.
     *
     * The second input argument variable_simplex_opt is an optional simplex that is in the domain
     * that's defined by the domain_tuples. For example, the domain_tuples for a function defined
     * over a triangle is the vector of three vertex tuples of the triangle. And the
     * variable_simplex, when exists, can be one of the three vertices. The function f can then be
     * differentiable wrt the variable_simplex.
     *
     * @tparam N The number of the tuples that consist of the domain of the function f(x)
     * @param domain_tuples The domain of the function f(x)
     * @param variable_simplex_opt The optional variable_simplex that the function f(x) is
     * differentiating wrt.
     * @return std::array<DSVec, N> The coordinates of the variables for the function f(x) that are
     * in autodiff type
     */
    std::vector<DSVec> get_coordinates(
        const simplex::Simplex& domain_simplex,
        const std::optional<simplex::Simplex>& variable_simplex_opt = {}) const;

    std::vector<DSVec> get_coordinates(
        const attribute::Accessor<double>& accessor,
        const simplex::Simplex& domain_simplex,
        const std::optional<simplex::Simplex>& variable_simplex_opt = {}) const;


    /**
     * @brief This function defines a function f(x) where f is defined over a simplex domain and
     * the variables for f are the n vertices coordinates of the simplex.
     *
     * The input coordinates are obtained through the get_coordinates function. They are
     * encoded in autodiff type so that the function can be differentiated through autodiff.
     *
     * @param coordinates
     * @return DScalar
     */
    virtual DScalar eval(
        const simplex::Simplex& domain_simplex,
        const std::vector<DSVec>& coordinates) const = 0;
};

} // namespace wmtk::function