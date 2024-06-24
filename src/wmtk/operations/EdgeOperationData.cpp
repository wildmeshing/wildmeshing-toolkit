
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include <wmtk/Mesh.hpp>
#include "EdgeOperationData.hpp"
namespace wmtk::operations {
auto EdgeOperationData::tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
    -> Tuple
{
    return m.tuple_from_id(type, gid);
}

void SplitAlternateFacetData::sort()
{
    std::sort(m_facet_maps.begin(), m_facet_maps.end());
}

// assumes the split cell map has been sorted
auto SplitAlternateFacetData::get_alternative_facets_it(const int64_t& input_cell) const
    -> AltData::const_iterator
{
    assert(std::is_sorted(m_facet_maps.begin(), m_facet_maps.end()));

    const auto sort_op = [&](const std::tuple<int64_t, std::array<int64_t, 2>>& value,
                             const int64_t& cell) -> bool { return std::get<0>(value) < cell; };


    auto it = std::lower_bound(m_facet_maps.begin(), m_facet_maps.end(), input_cell, sort_op);
    auto end = m_facet_maps.cend();
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

void CollapseAlternateFacetData::add(const Mesh& m, const Tuple& input_tuple) const
{
    m_data.emplace_back(m, input_tuple);
    // first tuple is different from the input by switching everything but vertex
    // second one is switch everything
}

CollapseAlternateFacetData::Data::Data(const Mesh& m, const Tuple& input_tuple)
    : input(input_tuple)
    , alts({{left_switches(m, input_tuple), right_switches(m, input_tuple)}})

{}
Tuple CollapseAlternateFacetData::Data::left_switches(const Mesh& m, const Tuple& t) const
{
    const auto switches =
        wmtk::utils::primitive_range(PrimitiveType::Edge, m.top_simplex_type() - 1);
}
Tuple CollapseAlternateFacetData::Data::right_switches(const Mesh& m, const Tuple& t) const
{
    //
}

} // namespace wmtk::operations
