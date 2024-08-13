#include "CollapseAlternateFacetOptionData.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/utils/TupleInspector.hpp>
namespace wmtk::operations::internal {

namespace {
auto make_opp_actions() -> std::array<int8_t, 4>
{
    std::array<int8_t, 4> darts;
    darts[0] = 0;
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        darts[j] = sd.opposite();
    }
    //
    // spdlog::error("Left Darts: {}", fmt::join(darts, ","));
    return darts;
}
auto make_right_ear_darts() -> std::array<int8_t, 4>
{
    return make_opp_actions();
}
auto make_left_ear_darts() -> std::array<int8_t, 4>
{
    auto darts = make_opp_actions();
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        int8_t& action = darts[j];
        action = sd.product(action, sd.primitive_as_index(wmtk::PrimitiveType::Vertex));
    }
    // spdlog::error("Right Darts: {}", fmt::join(darts, ","));
    return darts;
}
const static std::array<int8_t, 4> left_ear_darts = make_left_ear_darts();
const static std::array<int8_t, 4> right_ear_darts = make_right_ear_darts();

} // namespace
CollapseAlternateFacetOptionData::CollapseAlternateFacetOptionData(
    const Mesh& m,
    const autogen::SimplexDart& sd,
    const Tuple& input_tuple)
    : input(sd.dart_from_tuple(input_tuple))
    , alts({{left_switches(m, input_tuple), right_switches(m, input_tuple)}})
    , local_boundary_indices({{
          wmtk::utils::TupleInspector::local_id(input_tuple, m.top_simplex_type() - 1),
          wmtk::utils::TupleInspector::local_id(
              m.switch_tuple(input_tuple, PrimitiveType::Vertex),
              m.top_simplex_type() - 1),
      }})
{}

CollapseAlternateFacetOptionData::CollapseAlternateFacetOptionData(
    const Mesh& m,
    const Tuple& input_tuple)
    : CollapseAlternateFacetOptionData(
          m,
          autogen::SimplexDart::get_singleton(m.top_simplex_type()),
          input_tuple)
{}

auto CollapseAlternateFacetOptionData::left_switches(const Mesh& m, const Tuple& t) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    return get_neighbor_action(m, t, left_ear_darts[get_primitive_type_id(mesh_type)]);
}
auto CollapseAlternateFacetOptionData::right_switches(const Mesh& m, const Tuple& t) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    return get_neighbor_action(m, t, right_ear_darts[get_primitive_type_id(mesh_type)]);
}

auto CollapseAlternateFacetOptionData::get_neighbor_action(
    const Mesh& m,
    const Tuple& t,
    int8_t local_action) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    const PrimitiveType boundary_type = mesh_type - 1;
    Tuple r = wmtk::autogen::local_switch_tuple(mesh_type, t, local_action);
    Dart d;
    {
        const auto& sd = autogen::SimplexDart::get_singleton(m.top_simplex_type());
        spdlog::info(
            "Input dart {} being moved by action {} to {}",
            std::string(sd.dart_from_tuple(t)),
            local_action,
            std::string(sd.dart_from_tuple(r)));
    }
    if (!m.is_boundary(boundary_type, r)) {
        const auto& sd = autogen::SimplexDart::get_singleton(m.top_simplex_type());
        int8_t source_orientation = sd.valid_index_from_tuple(t);
        r = m.switch_tuple(r, m.top_simplex_type());
        d = sd.dart_from_tuple(r);
        int8_t& target_orientation = d.local_orientation();
        int8_t old = target_orientation;

        // encode the relative orientaiton at the d orientation
        target_orientation =
            autogen::find_local_dart_action(sd, source_orientation, target_orientation);
        spdlog::info(
            "Facet {} => {} Going from {} to {} and got {}",
            wmtk::utils::TupleInspector::as_string(t),
            wmtk::utils::TupleInspector::as_string(r),
            source_orientation,
            old,
            std::string(d));
    }

    return d;
}
} // namespace wmtk::operations::internal
