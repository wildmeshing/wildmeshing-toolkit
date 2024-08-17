#include "CollapseAlternateFacetData.hpp"
#include <array>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/multimesh/utils/find_local_dart_action.hpp>
#include <wmtk/multimesh/utils/find_local_switch_sequence.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "wmtk/autogen/SimplexDart.hpp"
#include "wmtk/utils/TupleInspector.hpp"

namespace wmtk::operations::internal {

void CollapseAlternateFacetData::add(const Mesh& m, const Tuple& input_tuple)
{
    m_data.emplace_back(m, input_tuple);
    // first tuple is different from the input by switching everything but vertex
    // second one is switch everything
}


CollapseAlternateFacetData::CollapseAlternateFacetData() = default;
CollapseAlternateFacetData::~CollapseAlternateFacetData() = default;

auto CollapseAlternateFacetData::get_alternative_data_it(const int64_t& input_facet) const
    -> AltData::const_iterator
{
    constexpr auto sort_op = [](const Data& value, const int64_t& facet_id) -> bool {
        return value.input.global_id() < facet_id;
    };
    auto it = std::lower_bound(m_data.begin(), m_data.end(), input_facet, sort_op);
    auto end = m_data.cend();

    // if we found

    // fix case where the lower bound was not the target value
    if (it != end && it->input.global_id() != input_facet) {
        it = end;
    }
    return it;
}
auto CollapseAlternateFacetData::get_alternatives_data(const Tuple& t) const -> const Data&
{
    auto it = get_alternative_data_it(t.m_global_cid);
    assert(it != m_data.cend());
    return *it;
}
std::array<Tuple, 2> CollapseAlternateFacetData::get_alternatives(
    const PrimitiveType mesh_pt,
    const Tuple& t,
    const PrimitiveType simplex_dimension) const
{
    const auto& data = get_alternatives_data(t);

    const wmtk::autogen::SimplexDart& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_pt);
    const wmtk::autogen::Dart t_dart = sd.dart_from_tuple(t);

    const int8_t action =
        wmtk::multimesh::utils::find_local_dart_action(mesh_pt, t_dart, data.input);
    auto map = [action, &sd, &data](const size_t index) -> Tuple {
        const wmtk::autogen::Dart& transform = data.alts[index];
        const int8_t& local_boundary_index = data.local_boundary_indices[index];
        const PrimitiveType mappable_dart_dimension = a;
        if (transform.is_null() || mappable_dart_dimension < simplex_dimension) {
            return {};
        } else {
            int8_t projected_subdart = sd.project_subdubdart(action, , mesh_pt - 1);
            int8_t mapped_dart = sd.product(tup.local_orientation(), action);
            const wmtk::autogen::Dart d(tup.global_id(), mapped_dart);
            return sd.tuple_from_dart(d);
        }
    };

    std::array<Tuple, 2> r{{map(0), map(1)}};

    return r;
}
Tuple CollapseAlternateFacetData::get_alternative(const PrimitiveType pt, const Tuple& t) const
{
    // TODO: map to a valid face


    auto alts = get_alternatives(pt, t);
    assert(!alts[0].is_null() || !alts[1].is_null());
    if (!alts[0].is_null()) {
        return alts[0];
    } else {
        return alts[1];
    }
    return {};
    //
}
} // namespace wmtk::operations::internal
