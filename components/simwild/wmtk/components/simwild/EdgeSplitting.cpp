#include "SimWildMesh.h"

#include <wmtk/utils/Logger.hpp>

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
        m_vertex_attribute[v_new].m_pos = to_rational(p);

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
        m_vertex_attribute[v_new].m_pos = to_rational(p);
        break;
    }
    return true;
}

} // namespace wmtk::components::simwild
