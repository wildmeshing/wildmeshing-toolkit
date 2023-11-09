#pragma once
#include <wmtk/function/utils/autodiff.h>
#include <optional>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
#include "PerSimplexDifferentiableFunction.hpp"
namespace wmtk::function {

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
    virtual DScalar eval(DSVec& coordinate0, DSVec& coordinates1, DSVec& coordinate2) const = 0;

    template <int N>
    std::array<DSVec, N> get_variable_coordinates(
        const std::vector<Tuple>& domain_tuples,
        const std::optional<Simplex>& variable_simplex_opt) const
    {
        ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());

        std::array<DSVec, N> coordinates;
        assert(N == domain_tuples.size());
        simplex::internal::SimplexEqualFunctor equal(mesh());
        for (int i = 0; i < domain_tuples.size(); i++) {
            Tuple domain_tuple = domain_tuples[i];
            if (variable_simplex_opt.has_value() &&
                equal(
                    Simplex(get_coordinate_attribute_primitive_type(), domain_tuple),
                    variable_simplex_opt.value())) {
                Simplex variable_simplex = variable_simplex_opt.value();
                coordinates[i] = utils::as_DScalar<DScalar>(
                    pos.const_vector_attribute(variable_simplex.tuple()));
                coordinates[i].conservativeResize(embedded_dimension());
            } else {
                Eigen::VectorXd temp_coord = pos.const_vector_attribute(domain_tuple);
                coordinates[i] = temp_coord.cast<DScalar>();
                coordinates[i].conservativeResize(embedded_dimension());
            }
        }
        return coordinates;
    }
};
} // namespace wmtk::function
