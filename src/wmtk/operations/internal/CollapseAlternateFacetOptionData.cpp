#include "CollapseAlternateFacetOptionData.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include "ear_actions.hpp"
namespace wmtk::operations::internal {

CollapseAlternateFacetOptionData::CollapseAlternateFacetOptionData(
    const Mesh& m,
    const autogen::SimplexDart& sd,
    const Tuple& input_tuple)
    : input(sd.dart_from_tuple(input_tuple))
    , alts({{left_switches(m, input_tuple), right_switches(m, input_tuple)}})
    , local_boundary_indices({{
          input_tuple.local_id(m.top_simplex_type() - 1),
          m.switch_tuple(input_tuple, PrimitiveType::Vertex).local_id(m.top_simplex_type() - 1),
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
    return get_neighbor_action(m, t, left_ear_action(mesh_type));
}
auto CollapseAlternateFacetOptionData::right_switches(const Mesh& m, const Tuple& t) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    return get_neighbor_action(m, t, right_ear_action(mesh_type));
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
    }

    return d;
}
} // namespace wmtk::operations::internal
