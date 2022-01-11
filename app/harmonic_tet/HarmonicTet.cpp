#include "HarmonicTet.hpp"

#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
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
        auto [weight, loc] = queue.front();
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
}
bool HarmonicTet::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    auto max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    wmtk::logger().debug("energy {} {}", edgeswap_cache.max_energy, max_energy);
    if (is_inverted(t) || is_inverted(*oppo_tet)) {
    wmtk::logger().debug("invert w/ energy {} {}", edgeswap_cache.max_energy, max_energy);
        return false;
    }
    if (max_energy > edgeswap_cache.max_energy) return false;
    wmtk::logger().debug("Going thru");
    return true;
}

void HarmonicTet::swap_all_faces(){};
bool HarmonicTet::swap_face_before(const Tuple& t)
{
    return true;
}
bool HarmonicTet::swap_face_after(const Tuple& t)
{
    return true;
}

void HarmonicTet::output_mesh(std::string file) const {
// warning: duplicate code.
    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i];
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
    msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

    msh.save(file, true);
}

} // namespace harmonic_tet