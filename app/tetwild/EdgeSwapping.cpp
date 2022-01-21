#include "TetWild.h"
#include "wmtk/utils/TupleUtils.hpp"

#include <wmtk/TetMesh.h>
#include <cassert>
#include <wmtk/utils/Logger.hpp>

auto measure_edge_length = [](auto& m, auto& l, auto& m_vertex_attribute) {
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m_vertex_attribute[v1.vid(m)].m_posf - m_vertex_attribute[v2.vid(m)].m_posf).squaredNorm();
    return length;
};

auto construct_queue =
    [](const tetwild::TetWild& m, const auto& m_vertex_attribute, const auto& tuples) {
        using namespace tetwild;

        wmtk::logger().debug("tuples.size() = {}", tuples.size());
        std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> ec_queue;

        for (auto& loc : tuples) {
            auto& v1 = loc;
            auto v2 = loc.switch_vertex(m);
            double length =
                (m_vertex_attribute[v1.vid(m)].m_posf - m_vertex_attribute[v2.vid(m)].m_posf)
                    .squaredNorm();
            ec_queue.emplace(loc, length);
        }
        return ec_queue;
    };

void tetwild::TetWild::swap_all_edges()
{
    auto cnt_suc = 0;
    auto& m = *this;
    auto queue = construct_queue(m, m_vertex_attribute, get_edges());
    while (!queue.empty()) {
        auto& [loc, weight] = queue.top();
        queue.pop();

        if (!loc.is_valid(m)) continue;
        auto length = measure_edge_length(m, loc, m_vertex_attribute);
        if (length != weight) continue;
        std::vector<wmtk::TetMesh::Tuple> newt;
        if (!swap_edge(loc, newt)) {
            continue;
        }
        auto new_edges = std::vector<wmtk::TetMesh::Tuple>();
        for (auto ti : newt) {
            for (auto j = 0; j < 6; j++) new_edges.push_back(tuple_from_edge(ti.tid(*this), j));
        }
        wmtk::unique_edge_tuples(m, new_edges);
        for (auto& e : new_edges) {
            auto& v1 = e;
            auto v2 = e.switch_vertex(m);
            double length =
                (m_vertex_attribute[v1.vid(m)].m_posf - m_vertex_attribute[v2.vid(m)].m_posf)
                    .squaredNorm();
            queue.emplace(e, length);
        }
        cnt_suc++;
    }
    wmtk::logger().debug("Edge Swapping Success {}", cnt_suc);
}

void tetwild::TetWild::swap_all_faces()
{
    auto cnt_suc = 0;
    auto& m = *this;
    auto queue = construct_queue(m, m_vertex_attribute, get_faces());
    while (!queue.empty()) {
        auto& [loc, weight] = queue.top();
        queue.pop();

        if (!loc.is_valid(*this)) continue;
        auto length = measure_edge_length(*this, loc, m_vertex_attribute);
        if (length != weight) continue;
        std::vector<wmtk::TetMesh::Tuple> newt;
        if (!swap_face(loc, newt)) {
            continue;
        }

        auto new_faces = std::vector<wmtk::TetMesh::Tuple>();
        for (auto ti : newt) {
            for (auto j = 0; j < 4; j++) new_faces.push_back(tuple_from_face(ti.tid(*this), j));
        }
        wmtk::unique_face_tuples(m, new_faces);
        for (auto& f : new_faces) { // the ordering for faces is not defined in the paper.
            auto& v1 = f;
            auto v2 = f.switch_vertex(m);
            double length =
                (m_vertex_attribute[v1.vid(m)].m_posf - m_vertex_attribute[v2.vid(m)].m_posf)
                    .squaredNorm();
            queue.emplace(f, length);
        }
        cnt_suc++;
    }
    wmtk::logger().debug("Face Swapping Success {}", cnt_suc);
}


bool tetwild::TetWild::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    edgeswap_cache.local().max_energy = max_energy;
    return true;
}
bool tetwild::TetWild::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    auto max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    if (is_inverted(t) || is_inverted(*oppo_tet)) {
        return false;
    }
    if (max_energy > edgeswap_cache.local().max_energy) return false;
    cnt_swap ++;
    return true;
}

bool tetwild::TetWild::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.local().max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    return true;
}

bool tetwild::TetWild::swap_face_after(const Tuple& t)
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
    wmtk::logger().trace("quality {} from {}", max_energy, edgeswap_cache.local().max_energy);

    if (max_energy > edgeswap_cache.local().max_energy) return false;

    cnt_swap ++;
    return true;
}
