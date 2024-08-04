#include "CollapseAlternateFacetData.hpp"
#include <spdlog/spdlog.h>
#include <array>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/multimesh/utils/find_local_dart_action.hpp>
#include <wmtk/multimesh/utils/find_local_switch_sequence.hpp>
#include <wmtk/multimesh/utils/local_switch_tuple.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "wmtk/autogen/SimplexDart.hpp"
#include "wmtk/multimesh/utils/find_local_dart_action.hpp"
#include "wmtk/utils/TupleInspector.hpp"

namespace wmtk::operations::internal {
class CollapseAlternateFacetData::Data
{
public:
    using Dart = autogen::Dart;
    Data(const Mesh& m, const Tuple& input_tuple);
    Data(const Mesh& m, const autogen::SimplexDart& sd, const Tuple& input_tuple);
    autogen::Dart input;

    // Stores {ear_global_id, M}
    // where M is defined by:
    // Let {G, O} be the input
    // Let {Ge, Oe} be the left/0 or right/1 ear
    // Let R be such that Oe = R O
    // We select an arbitrary dart that includes the right face
    // {G,Oa} a dart that includes the ear face, and
    // {Ge,Ob} a dart denoting a art on Ge whose D-1 subdart is equivalent
    // Let M be  Ob = M Oa.
    // Note that for D-1 subdart encoded on G, M will return the equivalent D-1 subdart on Ge
    //
    std::array<autogen::Dart, 2> alts;
    std::array<int8_t, 2> local_boundary_indices;

    // Let d be a dart where every D-simplex for D <the input mesh dimension
    // lies in left/index=0 (equivalently right/index=1) ear then
    // returns a dart such that those simplices are preserved using the other face
    //
    // This is given by the definition of alts and applying
    // Let {G, Od} be d
    // We compute {G, M Od}
    //
    Dart convert(const Dart& d, size_t index) const;


private:
    Dart left_switches(const Mesh& m, const Tuple& t) const;
    Dart right_switches(const Mesh& m, const Tuple& t) const;
    // given an ear tuple reports the relative orientation across the edge
    Dart get_neighbor_action(const Mesh& m, const Tuple& t, int8_t local_action) const;
};

namespace {
auto make_left_ear_darts() -> std::array<int8_t, 4>
{
    std::array<int8_t, 4> darts;
    darts[0] = 0;
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        int8_t& action = darts[j] = sd.identity();

        for (int8_t k = 0; k < j; ++k) {
            const PrimitiveType apt = get_primitive_type_from_id(k);
            action = sd.product(action, sd.primitive_as_index(apt));
        }
    }
    //
    return darts;
}
auto make_right_ear_darts() -> std::array<int8_t, 4>
{
    auto darts = make_left_ear_darts();
    for (int8_t j = 1; j < darts.size(); ++j) {
        PrimitiveType pt = get_primitive_type_from_id(j);
        wmtk::autogen::SimplexDart sd(pt);
        int8_t& action = darts[j];
        action = sd.product(action, sd.primitive_as_index(wmtk::PrimitiveType::Edge));
    }
    return darts;
}
const static std::array<int8_t, 4> left_ear_darts = make_left_ear_darts();
const static std::array<int8_t, 4> right_ear_darts = make_right_ear_darts();

} // namespace
void CollapseAlternateFacetData::add(const Mesh& m, const Tuple& input_tuple)
{
    m_data.emplace_back(m, input_tuple);
    // first tuple is different from the input by switching everything but vertex
    // second one is switch everything
}


CollapseAlternateFacetData::CollapseAlternateFacetData() = default;
CollapseAlternateFacetData::~CollapseAlternateFacetData() = default;
CollapseAlternateFacetData::Data::Data(
    const Mesh& m,
    const autogen::SimplexDart& sd,
    const Tuple& input_tuple)
    : input(sd.dart_from_tuple(input_tuple))
    , alts({{left_switches(m, input_tuple), right_switches(m, input_tuple)}})
    , local_boundary_indices({{
          wmtk::utils::TupleInspector::local_id(input_tuple, m.top_simplex_type() - 1),
          wmtk::utils::TupleInspector::local_id(input_tuple, m.top_simplex_type() - 1),
      }})
{}

CollapseAlternateFacetData::Data::Data(const Mesh& m, const Tuple& input_tuple)
    : Data(m, autogen::SimplexDart::get_singleton(m.top_simplex_type()), input_tuple)
{}

auto CollapseAlternateFacetData::Data::left_switches(const Mesh& m, const Tuple& t) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    return get_neighbor_action(m, t, left_ear_darts[get_primitive_type_id(mesh_type)]);
}
auto CollapseAlternateFacetData::Data::right_switches(const Mesh& m, const Tuple& t) const -> Dart
{
    const PrimitiveType mesh_type = m.top_simplex_type();
    return get_neighbor_action(m, t, right_ear_darts[get_primitive_type_id(mesh_type)]);
}

auto CollapseAlternateFacetData::Data::get_neighbor_action(
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
        int8_t source_orientation = sd.valid_index_from_tuple(r);
        r = m.switch_tuple(r, m.top_simplex_type());
        d = sd.dart_from_tuple(r);
        int8_t& target_orientation = d.local_orientation();

        // encode the relative orientaiton at the d orientation
        target_orientation =
            autogen::find_local_dart_action(sd, source_orientation, target_orientation);
    }

    return d;
}

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
    const Tuple& t) const
{
    const auto& data = get_alternatives_data(t);

    wmtk::autogen::SimplexDart sd(mesh_pt);
    const wmtk::autogen::Dart t_dart = sd.dart_from_tuple(t);

    const int8_t action =
        wmtk::multimesh::utils::find_local_dart_action(mesh_pt, t_dart, data.input);
    auto map = [action, &sd](const wmtk::autogen::Dart& tup) -> Tuple {
        if (tup.is_null()) {
            return {};
        } else {
            return sd.tuple_from_dart(local_dart_action(sd, tup, action));
        }
    };

    std::array<Tuple, 2> r{{map(data.alts[0]), map(data.alts[1])}};

    return r;
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
