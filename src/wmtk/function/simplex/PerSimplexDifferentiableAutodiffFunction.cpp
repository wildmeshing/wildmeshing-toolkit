#include "PerSimplexDifferentiableAutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
namespace wmtk::function {

PerSimplexDifferentiableAutodiffFunction::PerSimplexDifferentiableAutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType& domain_simplex_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexDifferentiableFunction(mesh, domain_simplex_type, variable_attribute_handle)
{}

PerSimplexDifferentiableAutodiffFunction::~PerSimplexDifferentiableAutodiffFunction() = default;

auto PerSimplexDifferentiableAutodiffFunction::get_coordinates(
    const Tuple& domain_tuple,
    const std::optional<Tuple>& variable_tuple_opt) const -> std::vector<DSVec>
{
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());
    return get_coordinates(pos, domain_tuple, variable_tuple_opt);
}
auto PerSimplexDifferentiableAutodiffFunction::get_coordinates(
    const ConstAccessor<double>& accessor,
    const Tuple& domain_tuple,
    const std::optional<Tuple>& variable_tuple_opt) const -> std::vector<DSVec>
{
    const PrimitiveType primitive_type = get_coordinate_attribute_primitive_type();
    const std::vector<Tuple> faces = wmtk::simplex::faces_single_dimension_tuples(
        mesh(),
        as_domain_simplex(domain_tuple),
        primitive_type);


    std::vector<DSVec> ret;
    ret.reserve(faces.size());

    std::transform(
        faces.begin(),
        faces.end(),
        std::back_inserter(ret),
        [&](const Tuple& face_tuple) -> DSVec {
            auto value = accessor.const_vector_attribute(face_tuple).eval();
            // if we have a variable simplex and are trying to differentiate it then fill its
            // gradient
            if (variable_tuple_opt.has_value()) {
                const auto& variable_tuple = variable_tuple_opt.value();
                if (wmtk::simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        primitive_type,
                        face_tuple,
                        variable_tuple)) {
                    return utils::as_DScalar<DScalar>(value);
                }
            }
            return value.cast<DScalar>();
        });
    return ret;
}
auto PerSimplexDifferentiableAutodiffFunction::get_coordinates(
    const Simplex& domain_simplex,
    const std::optional<Simplex>& variable_simplex_opt) const -> std::vector<DSVec>
{
    assert(domain_simplex.primitive_type() == get_domain_simplex_type());
    if (variable_simplex_opt.has_value()) {
        assert(
            variable_simplex_opt.value().primitive_type() ==
            get_coordinate_attribute_primitive_type());
        return get_coordinates(domain_simplex.tuple(), variable_simplex_opt.value().tuple());
    } else {
        return get_coordinates(domain_simplex.tuple());
    }
}

} // namespace wmtk::function
