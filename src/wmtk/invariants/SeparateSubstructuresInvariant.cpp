#include "SeparateSubstructuresInvariant.hpp"

#include <algorithm>
#include <set>
#include <wmtk/Mesh.hpp>

namespace wmtk::invariants {

SeparateSubstructuresInvariant::SeparateSubstructuresInvariant(
    const Mesh& m,
    bool check_condition_2)
    : Invariant(m, true, false, false)
    , m_check_condition_2(check_condition_2)
{}

bool SeparateSubstructuresInvariant::before(const simplex::Simplex& s) const
{
    throw std::runtime_error(
        "Pointless invariant withouth multimesh. Bring back for tag multimesh");

    // assert(s.primitive_type() == PrimitiveType::Edge);
    //
    // const Tuple& t = s.tuple();
    //
    // const simplex::Simplex& e = s;
    // const simplex::Simplex v0 = simplex::Simplex::vertex(mesh(), t);
    // const simplex::Simplex v1 =
    //     simplex::Simplex::vertex(mesh(), mesh().switch_tuple(t, PrimitiveType::Vertex));
    //
    // const Mesh& m = mesh();
    //
    // const auto substructures = m.get_all_child_meshes();
    // std::vector<std::vector<int64_t>> subs_e;
    // std::vector<std::vector<int64_t>> subs_v0;
    // std::vector<std::vector<int64_t>> subs_v1;
    // subs_e.reserve(substructures.size());
    // subs_v0.reserve(substructures.size());
    // subs_v1.reserve(substructures.size());
    //
    // for (const auto& sub : substructures) {
    //     const std::vector<int64_t> id = sub->absolute_multi_mesh_id();
    //
    //     if (!m.map_to_child_tuples(*sub, e).empty()) {
    //         subs_e.emplace_back(id);
    //     }
    //     if (!m.map_to_child_tuples(*sub, v0).empty()) {
    //         subs_v0.emplace_back(id);
    //     }
    //     if (!m.map_to_child_tuples(*sub, v1).empty()) {
    //         subs_v1.emplace_back(id);
    //     }
    // }
    // std::sort(subs_e.begin(), subs_e.end());
    // std::sort(subs_v0.begin(), subs_v0.end());
    // std::sort(subs_v1.begin(), subs_v1.end());
    //
    //// Condition 1: The intersection of subs(V0) and subs(V1) is a subset of subs(E).
    // std::vector<std::vector<int64_t>> subs_v0v1;
    // std::set_intersection(
    //     subs_v0.begin(),
    //     subs_v0.end(),
    //     subs_v1.begin(),
    //     subs_v1.end(),
    //     std::back_inserter(subs_v0v1));
    //
    // if (!std::includes(subs_e.begin(), subs_e.end(), subs_v0v1.begin(), subs_v0v1.end())) {
    //     return false;
    // }
    //
    // if (!m_check_condition_2) {
    //     return true;
    // }
    //
    //// Condition 2: subs(V0) is a subset of subs(V1) or subs(V1) is a subset of subs(V0).
    // const bool v1_includes_v0 =
    //     std ::includes(subs_v1.begin(), subs_v1.end(), subs_v0.begin(), subs_v0.end());
    // const bool v0_includes_v1 =
    //     std ::includes(subs_v0.begin(), subs_v0.end(), subs_v1.begin(), subs_v1.end());
    // return v0_includes_v1 || v1_includes_v0;
}

} // namespace wmtk::invariants