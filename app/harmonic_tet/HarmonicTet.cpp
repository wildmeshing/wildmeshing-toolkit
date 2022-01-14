#include "HarmonicTet.hpp"

#include <igl/predicates/predicates.h>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

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
    auto& m = *this;
    while (!queue.empty()) {
        auto [weight, loc] = queue.front();
        queue.pop();

        if (!loc.is_valid(*this)) continue;
        wmtk::TetMesh::Tuple newt;
        if (!swap_edge(loc, newt)) {
            continue;
        }
        auto new_edges = std::vector<wmtk::TetMesh::Tuple>();
        assert(newt.switch_tetrahedron(m));
        for (auto ti : {newt.tid(m), newt.switch_tetrahedron(m)->tid(m)}) {
            for (auto j = 0; j < 6; j++) new_edges.push_back(tuple_from_edge(ti, j));
        }
        wmtk::unique_edge_tuples(m, new_edges);
        for (auto& e : new_edges) {
            queue.emplace(-1., e);
        }
        cnt_suc++;
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
    return true;
}

void HarmonicTet::swap_all_faces()
{
    auto queue = std::queue<std::tuple<bool, wmtk::TetMesh::Tuple>>();

    for (auto& loc : get_faces()) {
        double length = -1.;
        queue.emplace(length, loc);
    }
    auto& m = *this;
    auto cnt_suc = 0;
    while (!queue.empty()) {
        auto [weight, loc] = queue.front();
        queue.pop();

        if (!loc.is_valid(m)) continue;
        wmtk::TetMesh::Tuple newt;
        if (!swap_face(loc, newt)) {
            continue;
        }
        auto new_tets = std::vector<size_t>(1, newt.tid(m));
        for (auto k = 0; k < 2; k++) {
            newt = newt.switch_face(m);
            newt = newt.switch_tetrahedron(m).value();
            new_tets.push_back(newt.tid(m));
        }

        auto new_faces = std::vector<wmtk::TetMesh::Tuple>();
        for (auto ti : new_tets) {
            for (auto j = 0; j < 4; j++) new_faces.push_back(tuple_from_face(ti, j));
        }
        wmtk::unique_face_tuples(m, new_faces);
        for (auto& e : new_faces) {
            queue.emplace(-1., e);
        }
        cnt_suc++;
    }
};
bool HarmonicTet::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    return true;
}

bool HarmonicTet::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
    }
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, edgeswap_cache.max_energy);

    if (max_energy > edgeswap_cache.max_energy) return false;
    return true;
}

void HarmonicTet::output_mesh(std::string file) const
{
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