#include "SimWildMesh.h"

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>

namespace wmtk::components::simwild {

bool SimWildMesh::split_before_cells(const Tuple& edge_tuple, const std::vector<Tuple>& parents)
{
    auto& cache = split_tag_cache.local();
    cache.tets.clear();

    const simplex::Edge edge = simplex_from_edge(edge_tuple);
    for (const Tuple& parent : parents) {
        const simplex::Tet tet = simplex_from_tet(parent);
        cache.tets[tet.opposite_edge(edge)] = m_tet_attribute.at(parent.tid(*this));
    }
    return true;
}

bool SimWildMesh::split_after_cells(
    const size_t v1,
    const size_t v2,
    const size_t v_new,
    const std::vector<Tuple>& children)
{
    auto& cache = split_tag_cache.local();
    cache.v_new = v_new;

    const auto children1 = get_incident_tets_for_edge(v1, v_new);
    const simplex::Edge edge1(v1, v_new);
    for (const Tuple& child : children1) {
        const simplex::Edge opposite = simplex_from_tet(child).opposite_edge(edge1);
        const auto it = cache.tets.find(opposite);
        if (it == cache.tets.end()) return false;
        m_tet_attribute[child.tid(*this)] = it->second;
    }

    const auto children2 = get_incident_tets_for_edge(v2, v_new);
    const simplex::Edge edge2(v2, v_new);
    for (const Tuple& child : children2) {
        const simplex::Edge opposite = simplex_from_tet(child).opposite_edge(edge2);
        const auto it = cache.tets.find(opposite);
        if (it == cache.tets.end()) return false;
        m_tet_attribute[child.tid(*this)] = it->second;
    }

    return children1.size() + children2.size() == children.size();
}

bool SimWildMesh::split_adjust_position(const size_t v_new, const std::vector<Tuple>& children)
{
    if (m_edge_split_at) {
        // exactly at the fraction asked for: rounded if that inverts no child, else exact
        const auto& cache = split_cache.local();
        const size_t from = m_edge_split_at->from;
        if (from != cache.v1_id && from != cache.v2_id) {
            log_and_throw_error(
                "split_adjust_position: vertex {} is no end of the edge ({}, {})",
                from,
                cache.v1_id,
                cache.v2_id);
        }
        const size_t to = from == cache.v1_id ? cache.v2_id : cache.v1_id;
        const Vector3r a = m_vertex_attribute[from].pos();
        const Vector3r b = m_vertex_attribute[to].pos();
        Vector3r exact;
        for (int k = 0; k < 3; ++k) {
            exact[k] = a[k] + (b[k] - a[k]) * m_edge_split_at->fraction;
        }
        VertexAttributes& va = m_vertex_attribute[v_new];
        const auto inverts = [&] {
            return std::any_of(children.begin(), children.end(), [&](const Tuple& t) {
                return is_inverted(t);
            });
        };
        va.m_posf = to_double(exact);
        va.m_is_rounded = true;
        va.set_pos_to_posf();
        if (inverts()) {
            va.set_pos(exact);
            va.m_is_rounded = false;
            m_all_rounded.store(false, std::memory_order_relaxed);
            if (inverts()) {
                log_and_throw_error(
                    "split_adjust_position: the exact split point of ({}, {}) inverts a tet",
                    cache.v1_id,
                    cache.v2_id);
            }
        }
        return true;
    }
    if (!m_voronoi_split_fn || !m_vertex_attribute[v_new].m_is_rounded) return true;

    const auto& shared_cache = split_cache.local();
    const size_t v1 = shared_cache.v1_id;
    const size_t v2 = shared_cache.v2_id;
    auto& p = m_vertex_attribute[v_new].m_posf;

    Vector3d p0 = m_vertex_attribute[v1].m_posf;
    Vector3d p1 = m_vertex_attribute[v2].m_posf;
    if (m_voronoi_split_fn(p0) >= 0) std::swap(p0, p1);

    for (int i = 0; i < 20; ++i) {
        p = 0.5 * (p0 + p1);
        m_vertex_attribute[v_new].set_pos_to_posf(); // p aliases m_posf

        bool inverted = false;
        for (const Tuple& child : children) {
            if (is_inverted(child)) {
                inverted = true;
                break;
            }
        }
        if (inverted || (p1 - p0).squaredNorm() < 1e-20) break;

        if (m_voronoi_split_fn(p) < 0) {
            p0 = p;
        } else {
            p1 = p;
        }
    }

    for (const Tuple& child : children) {
        if (!is_inverted(child)) continue;
        logger().warn("Voronoi split inverted a cell; reverting to the TetWild midpoint");
        p = 0.5 * (m_vertex_attribute[v1].m_posf + m_vertex_attribute[v2].m_posf);
        m_vertex_attribute[v_new].set_pos_to_posf(); // p aliases m_posf
        break;
    }
    return true;
}

} // namespace wmtk::components::simwild
