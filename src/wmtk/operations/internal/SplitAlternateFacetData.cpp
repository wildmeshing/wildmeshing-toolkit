#include "SplitAlternateFacetData.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "ear_actions.hpp"
namespace wmtk::operations::internal {
namespace {
constexpr auto sort_op = [](const SplitAlternateFacetData::Data& a,
                            const SplitAlternateFacetData::Data& b) -> bool {
    return a.input.global_id() < b.input.global_id();
} constexpr auto sort_int_op = [](const SplitAlternateFacetData::Data& value,
                                  const int64_t& facet_id) -> bool {
    return value.input.global_id() < facet_id;
};
} // namespace

void SplitAlternateFacetData::sort()
{
    std::sort(m_facet_maps.begin(), m_facet_maps.end(), sort_op);
}

// assumes the split cell map has been sorted
auto SplitAlternateFacetData::get_alternative_facets_it(const int64_t& input_cell) const
    -> AltData::const_iterator
{
    assert(std::is_sorted(m_facet_maps.begin(), m_facet_maps.end(), sort_op));


    auto it = std::lower_bound(m_facet_maps.begin(), m_facet_maps.end(), input_cell, sort_int_op);
    auto end = m_facet_maps.cend();
    // fix case where the lower bound was not the target value
    if (it != end && it->input.global_id() != input_cell) {
        it = end;
    }
    return it;
}

auto SplitAlternateFacetData::get_alternative_facets(const int64_t& input_cell) const
    -> const std::array<int64_t, 2>&
{
    auto it = get_alternative_facets_it(input_cell);
    assert(it != m_facet_maps.cend());
    return std::get<1>(*it);
}
auto SplitAlternateFacetData::get_alternative(
    const PrimitiveType mesh_pt,
    const Tuple& t,
    const PrimitiveType simplex_dimension) const -> Tuple
{
    assert(mesh_pt > PrimitiveType::Vertex);
    auto alts = get_alternative_facets(wmtk::utils::TupleInspector::global_cid(t));
    int64_t new_global_cid = -1;

    const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_pt);

    wmtk::autogen::Dart dart = sd.dart_from_tuple(t);

    const PrimitiveType boundary_type = mesh_pt - 1;
    auto get_ear_max_subdart_size = [&](int8_t action) {
        const int8_t orientation = sd.product(action, dart.local_orientation());
        const int8_t simplex = sd.simplex_index(orientation, boundary_type);
    };

    const int8_t left_ear_max_subdart_size = get_ear_max_subdart_size(left_ear_action(mesh_pt));
    const int8_t right_ear_max_subdart_size = get_ear_max_subdart_size(right_ear_action(mesh_pt));


    else if (int8_t preserved_subdarts_to_face = 0; preserved_subdarts_to_face > 0)
    { // TODO: make this do something

        new_global_cid = alts[0];
    }
    else
    {
        new_global_cid = alts[1];
    }

    return {
        wmtk::utils::TupleInspector::local_vid(t),
        wmtk::utils::TupleInspector::local_eid(t),
        wmtk::utils::TupleInspector::local_fid(t),
        new_global_cid};
}
} // namespace wmtk::operations::internal
