#include "HarmonicTet.hpp"

#include <wmtk/utils/TetraQualityUtils.hpp>

#include <igl/predicates/predicates.h>

#include <queue>

namespace harmonic_tet {


double HarmonicTet::get_quality(const Tuple& loc)
{
    Eigen::MatrixXd ps(4, 3);
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps.row(j) = m_vertex_attribute[tups[j].vid(*this)];
    }
    return wmtk::harmonic_energy(ps);
}


bool HarmonicTet::is_inverted(const Tuple& loc)
{
    std::array<Eigen::Vector3d, 4> ps;
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[tups[j].vid(*this)];
    }

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false; // extremely annoying.
    return true;
}

void HarmonicTet::swap_all_edges()
{
    auto queue = std::queue<std::tuple<double, wmtk::TetMesh::Tuple>>();

    for (auto& loc : get_edges()) {
        double length = -1.;
        queue.emplace(length, loc);
    }

    auto cnt_suc = 0;
    while (!queue.empty()) {
        auto& [weight, loc] = queue.front();
        queue.pop();

        if (!loc.is_valid(*this)) continue;
        if (!swap_edge(loc)) {
            continue;
        }
        cnt_suc++;
        // not pushing back.
    }
}

bool HarmonicTet::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    edgeswap_cache.max_energy = max_energy;
    return true;
};
bool HarmonicTet::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    auto max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    if (is_inverted(t) || is_inverted(*oppo_tet)) {
        return false;
    }
    if (max_energy > edgeswap_cache.max_energy) return false;
    return true;
};

void HarmonicTet::swap_all_faces(){};
bool HarmonicTet::swap_face_before(const Tuple& t)
{
    return true;
};
bool HarmonicTet::swap_face_after(const Tuple& t)
{
    return true;
};

} // namespace harmonic_tet