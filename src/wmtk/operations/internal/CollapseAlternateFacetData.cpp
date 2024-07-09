#include "CollapseAlternateFacetData.hpp"
#include <array>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/find_local_switch_sequence.hpp>
#include <wmtk/multimesh/utils/local_switch_tuple.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::operations::internal {
void CollapseAlternateFacetData::add(const Mesh& m, const Tuple& input_tuple)
{
    m_data.emplace_back(m, input_tuple);
    // first tuple is different from the input by switching everything but vertex
    // second one is switch everything
}

CollapseAlternateFacetData::Data::Data(const Mesh& m, const Tuple& input_tuple)
    : input(input_tuple)
    , alts({{left_switches(m, input_tuple), right_switches(m, input_tuple)}})

{}
Tuple CollapseAlternateFacetData::Data::left_switches(const Mesh& m, const Tuple& t)
{
    const PrimitiveType boundary_type = m.top_simplex_type() - 1;
    Tuple r = t;
    if (m.top_simplex_type() > PrimitiveType::Edge) {
        const auto switches = wmtk::utils::primitive_range(PrimitiveType::Edge, boundary_type);
        r = m.switch_tuples_unsafe(t, switches);
    }
    if (m.is_boundary(boundary_type, r)) {
        r = Tuple{};
    } else {
        r = m.switch_tuple(r, m.top_simplex_type());
    }
    return r;
}
Tuple CollapseAlternateFacetData::Data::right_switches(const Mesh& m, const Tuple& t)
{
    const PrimitiveType boundary_type = m.top_simplex_type() - 1;
    const auto switches = wmtk::utils::primitive_range(PrimitiveType::Vertex, boundary_type);
    auto r = m.switch_tuples_unsafe(t, switches);
    if (m.is_boundary(boundary_type, r)) {
        r = Tuple{};
    } else {
        r = m.switch_tuple(r, m.top_simplex_type());
    }
    return r;
}

auto CollapseAlternateFacetData::get_alternative_data_it(const int64_t& input_facet) const
    -> AltData::const_iterator
{
    constexpr auto sort_op = [](const Data& value, const int64_t& facet_id) -> bool {
        return value.input.m_global_cid < facet_id;
    };
    auto it = std::lower_bound(m_data.begin(), m_data.end(), input_facet, sort_op);
    auto end = m_data.cend();

    // if we found

    // fix case where the lower bound was not the target value
    if (it != end && it->input.m_global_cid != input_facet) {
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
    const Tuple& t) const
{
    const auto& data = get_alternatives_data(t);

    const std::vector<PrimitiveType> sequence =
        wmtk::multimesh::utils::find_local_switch_sequence(t, data.input, mesh_pt);
    auto map = [&mesh_pt, &sequence](const Tuple& tup) -> Tuple {
        if (tup.is_null()) {
            return tup;
        } else {
            return wmtk::multimesh::utils::local_switch_tuples(mesh_pt, tup, sequence);
        }
    };

    return std::array<Tuple, 2>{{map(data.alts[0]), map(data.alts[1])}};
}
Tuple CollapseAlternateFacetData::get_alternative(const PrimitiveType pt, const Tuple& t) const
{
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
