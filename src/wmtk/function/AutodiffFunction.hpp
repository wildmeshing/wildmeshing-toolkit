#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <optional>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "PerSimplexDifferentiableFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is an extension of the PerSimplexDifferentiableFunction class that uses autodiff
 * encoding for differentiations
 *
 */
class AutodiffFunction : public PerSimplexDifferentiableFunction
{
public:
    using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
    using Scalar = typename DScalar::Scalar;
    using DSVec = Eigen::VectorX<DScalar>;
    static_assert(
        std::is_same_v<Scalar, double>); // MTAO: i'm leaving scalar here but is it ever not double?
    AutodiffFunction(
        const Mesh& mesh,
        const PrimitiveType& domain_simplex_type,
        const attribute::MeshAttributeHandle<double>& variable_attribute_handle);

    ~AutodiffFunction();

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
        const Tuple& domain_tuple,
        const std::optional<Tuple>& variable_tuple_opt = {}) const;

    std::vector<DSVec> get_coordinates(
        const Simplex& domain_simplex,
        const std::optional<Simplex>& variable_simplex_opt = {}) const;

    template <int N>
    std::array<DSVec, N> get_coordinates_T(
        const Tuple& domain_tuple,
        const std::optional<Tuple>& variable_tuple_opt = {}) const;

    template <int N>
    std::array<DSVec, N> get_coordinates_T(
        const Simplex& domain_simplex,
        const std::optional<Simplex>& variable_simplex_opt = {}) const;
};

template <int N>
auto AutodiffFunction::get_coordinates_T(
    const Tuple& domain_tuple,
    const std::optional<Tuple>& variable_tuple_opt) const -> std::array<DSVec, N>
{
    auto vec = get_coordinates(domain_tuple, variable_tuple_opt);
    assert(vec.size() == N);
    std::array<DSVec, N> r;
    std::copy(vec.begin(), vec.end(), r.begin());
    return r;
}
template <int N>
auto AutodiffFunction::get_coordinates_T(
    const Simplex& domain_simplex,
    const std::optional<Simplex>& variable_simplex_opt) const -> std::array<DSVec, N>
{
    auto vec = get_coordinates(domain_simplex, variable_simplex_opt);
    assert(vec.size() == N);
    std::array<DSVec, N> r;
    std::copy(vec.begin(), vec.end(), r.begin());
    return r;
}
} // namespace wmtk::function
