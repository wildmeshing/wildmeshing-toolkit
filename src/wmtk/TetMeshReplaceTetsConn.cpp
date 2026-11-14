#include <wmtk/TetMesh.h>

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <string>

namespace wmtk {

size_t TetMesh::add_vertex_slot()
{
    if (m_surface_history || m_curve_history) {
        log_and_throw_error(
            "TetMesh::add_vertex_slot: no history hooks; it cannot run while a surface or curve "
            "history is recording");
    }
    reserve_free_slots(0, 1);
    const size_t v = get_next_empty_slot_v();
    if (v == INVALID_SLOT) {
        log_and_throw_error("TetMesh::add_vertex_slot: no vertex slot left");
    }
    m_vertex_connectivity[v].m_conn_tets.clear();
    m_vertex_connectivity[v].m_is_removed = false;
    m_unplaced_vertex_slots.push_back(v);
    return v;
}

bool TetMesh::replace_tets(
    const std::vector<size_t>& old_tids,
    const std::vector<std::array<size_t, 4>>& new_tets,
    std::vector<size_t>& new_tids)
{
    if (m_surface_history || m_curve_history) {
        log_and_throw_error(
            "TetMesh::replace_tets: no history hooks; it cannot run while a surface or curve "
            "history is recording");
    }
    new_tids.clear();
    // a refusal changes nothing but releases the slots add_vertex_slot() handed out
    const auto refuse = [this](const std::string& why) {
        logger().warn("TetMesh::replace_tets refused: {}", why);
        for (const size_t v : m_unplaced_vertex_slots) {
            m_vertex_connectivity[v].m_conn_tets.clear();
            m_vertex_connectivity[v].m_is_removed = true;
        }
        m_unplaced_vertex_slots.clear();
        return false;
    };

    // the old tets: distinct, and in the mesh
    const std::set<size_t> old_set(old_tids.begin(), old_tids.end());
    if (old_set.size() != old_tids.size()) {
        return refuse("an old tet is listed twice");
    }
    for (const size_t tid : old_tids) {
        if (tid >= tet_capacity() || m_tet_connectivity[tid].m_is_removed) {
            return refuse("an old tet is not in the mesh");
        }
    }

    // the new tets: four distinct vertices each, of the mesh or slots, every slot used
    const std::set<size_t> unplaced(m_unplaced_vertex_slots.begin(), m_unplaced_vertex_slots.end());
    std::set<size_t> placed;
    for (const std::array<size_t, 4>& t : new_tets) {
        for (int i = 0; i < 4; ++i) {
            const size_t v = t[i];
            if (v >= vert_capacity() || m_vertex_connectivity[v].m_is_removed) {
                return refuse("a new tet has a vertex that is not in the mesh");
            }
            for (int j = i + 1; j < 4; ++j) {
                if (t[j] == v) {
                    return refuse("a new tet has a vertex twice");
                }
            }
            if (unplaced.count(v) != 0) {
                placed.insert(v);
            } else if (m_vertex_connectivity[v].m_conn_tets.empty()) {
                return refuse("a new tet has a vertex in no tet that add_vertex_slot did not make");
            }
        }
    }
    if (placed.size() != unplaced.size()) {
        return refuse("a vertex slot from add_vertex_slot is in no new tet");
    }

    // The faces, oriented by each tet's vertex order (the boundary of (a, b, c, d) is (b, c, d),
    // (a, d, c), (a, b, d), (a, c, b)), counted by orientation: old +, old -, new +, new -. An
    // orientation is + if, rotated to start at its smallest vertex, the face's second vertex is
    // smaller than its third.
    std::map<std::array<size_t, 3>, std::array<int, 4>> faces;
    const auto count_faces = [&faces](const std::array<size_t, 4>& t, const int first) {
        static constexpr int local[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
        for (const auto& l : local) {
            std::array<size_t, 3> f{{t[l[0]], t[l[1]], t[l[2]]}};
            std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
            const bool positive = f[1] < f[2];
            std::sort(f.begin(), f.end());
            ++faces[f][first + (positive ? 0 : 1)];
        }
    };
    for (const size_t tid : old_tids) {
        count_faces(m_tet_connectivity[tid].m_indices, 0);
    }
    for (const std::array<size_t, 4>& t : new_tets) {
        count_faces(t, 2);
    }
    // the tets outside the old ones holding a face
    const auto tets_outside_on = [&](const std::array<size_t, 3>& f) {
        std::vector<size_t> common = m_vertex_connectivity[f[0]].m_conn_tets;
        common = set_intersection(common, m_vertex_connectivity[f[1]].m_conn_tets);
        common = set_intersection(common, m_vertex_connectivity[f[2]].m_conn_tets);
        size_t n = 0;
        for (const size_t tid : common) {
            n += old_set.count(tid) == 0 ? 1 : 0;
        }
        return n;
    };
    for (const auto& [f, n] : faces) {
        if (n[2] > 1 || n[3] > 1) {
            return refuse("a face is in more than two new tets, or in two the same way round");
        }
        if (n[2] - n[3] != n[0] - n[1]) {
            return refuse("the new tets' boundary is not the old tets'");
        }
        if (n[2] == 1 && n[3] == 1 && tets_outside_on(f) != 0) {
            return refuse("a face inside the region is a face of a tet outside it");
        }
    }

    // the old tets' slots first, then new ones
    if (new_tets.size() > old_tids.size()) {
        reserve_free_slots(new_tets.size() - old_tids.size(), 0);
    }
    std::vector<size_t> ids(new_tets.size());
    for (size_t i = 0; i < new_tets.size(); ++i) {
        ids[i] = i < old_tids.size() ? old_tids[i] : get_next_empty_slot_t();
        if (ids[i] == INVALID_SLOT) {
            for (size_t j = old_tids.size(); j < i; ++j) {
                m_tet_connectivity[ids[j]].m_is_removed = true;
            }
            return refuse("no tet slot left");
        }
    }

    std::vector<size_t> touched;
    for (const size_t tid : old_tids) {
        for (const size_t v : m_tet_connectivity[tid].m_indices) {
            vector_erase(m_vertex_connectivity[v].m_conn_tets, tid);
            touched.push_back(v);
        }
    }
    for (size_t i = new_tets.size(); i < old_tids.size(); ++i) {
        m_tet_connectivity[old_tids[i]].m_is_removed = true;
        m_tet_connectivity[old_tids[i]].hash++;
    }
    for (size_t i = 0; i < new_tets.size(); ++i) {
        TetrahedronConnectivity& tet = m_tet_connectivity[ids[i]];
        tet.m_indices = new_tets[i];
        tet.m_is_removed = false;
        tet.hash++;
        for (const size_t v : new_tets[i]) {
            m_vertex_connectivity[v].m_conn_tets.push_back(ids[i]);
            touched.push_back(v);
        }
    }
    vector_unique(touched);
    for (const size_t v : touched) {
        std::vector<size_t>& conn = m_vertex_connectivity[v].m_conn_tets;
        std::sort(conn.begin(), conn.end());
        conn.erase(std::unique(conn.begin(), conn.end()), conn.end());
        if (conn.empty()) {
            m_vertex_connectivity[v].m_is_removed = true; // an old vertex no new tet uses
        }
    }
    m_unplaced_vertex_slots.clear();
    new_tids = std::move(ids);
    return true;
}

} // namespace wmtk
