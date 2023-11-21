#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <optional>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include "PerSimplexDifferentiableFunction.hpp"
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
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
     * @brief This function defines a function f(x) where f is defined over a triangle domain and
     * the variables for f are the three vertices coordinates of the triangle.
     *
     * The input three coordinates are obtained through the get_variable_coordinates function. Theya
     * re encoded in autodiff type so that the function can be differentiated through autodiff.
     *
     * @param coordinate0
     * @param coordinate1
     * @param coordinate2
     * @return DScalar
     */
    virtual DScalar eval(DSVec& coordinate0, DSVec& coordinate1, DSVec& coordinate2) const = 0;

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
    template <int N>
    std::array<DSVec, N> get_variable_coordinates(
        const std::vector<Tuple>& domain_tuples,
        const std::optional<Simplex>& variable_simplex_opt) const;
};

template <int N>
std::array<AutodiffFunction::DSVec, N> AutodiffFunction::get_variable_coordinates(
    const std::vector<Tuple>& domain_tuples,
    const std::optional<Simplex>& variable_simplex_opt) const
{
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());

    std::array<DSVec, N> coordinates;
    assert(N == domain_tuples.size());
    for (size_t i = 0; i < domain_tuples.size(); i++) {
        Tuple domain_tuple = domain_tuples[i];
        Simplex domain_simplex(get_coordinate_attribute_primitive_type(), domain_tuple);
        if (variable_simplex_opt.has_value() &&
            wmtk::simplex::utils::SimplexComparisons::equal(mesh(),domain_simplex, variable_simplex_opt.value())) {
            Simplex variable_simplex = variable_simplex_opt.value();
            coordinates[i] =
                utils::as_DScalar<DScalar>(pos.const_vector_attribute(variable_simplex.tuple()));
        } else {
            Eigen::VectorXd temp_coord = pos.const_vector_attribute(domain_tuple);
            coordinates[i] = temp_coord.cast<DScalar>();
        }
    }
    return coordinates;
}
} // namespace wmtk::function
