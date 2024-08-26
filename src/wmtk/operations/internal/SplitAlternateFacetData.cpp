#include "SplitAlternateFacetData.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/TupleInspector.hpp>
namespace wmtk::operations::internal {

void SplitAlternateFacetData::sort()
{
    std::sort(m_facet_maps.begin(), m_facet_maps.end());
}

// assumes the split cell map has been sorted
auto SplitAlternateFacetData::get_alternative_facets_it(const int64_t& input_cell) const
    -> AltData::const_iterator
{
    assert(std::is_sorted(m_facet_maps.begin(), m_facet_maps.end()));

    constexpr auto sort_op = [](const std::tuple<int64_t, std::array<int64_t, 2>>& value,
                                const int64_t& cell) -> bool { return std::get<0>(value) < cell; };


    auto it = std::lower_bound(m_facet_maps.begin(), m_facet_maps.end(), input_cell, sort_op);
    auto end = m_facet_maps.cend();
    // fix case where the lower bound was not the target value
    if (it != end && std::get<0>(*it) != input_cell) {
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
        const PrimitiveType simplex_dimension) const -> Tuple {

        auto alts = get_alternative_facets(wmtk::utils::TupleInspector::global_cid(t));
        int64_t new_global_cid = -1;
        if(alts[0] == -1 || alts[1] == -1) {
            new_global_cid = alts[0] == -1 ? alts[1] : alts[0];
        } else if(int8_t preserved_subdarts_to_face = 0; preserved_subdarts_to_face > 0) { // TODO: make this do something

            new_global_cid = alts[0];
        } else {
            new_global_cid = alts[1];
        }

        return {
                wmtk::utils::TupleInspector::local_vid(t),
                wmtk::utils::TupleInspector::local_eid(t),
                wmtk::utils::TupleInspector::local_fid(t),
                new_global_cid};

    }
} // namespace wmtk::operations::internal
