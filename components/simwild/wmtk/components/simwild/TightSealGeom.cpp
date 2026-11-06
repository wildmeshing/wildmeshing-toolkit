#include "TightSealGeom.hpp"

#include "SimWildMesh.h"
#include "SimWildMeshTri.hpp"

#include <wmtk/utils/ConsistentTopology.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_set>

namespace wmtk::components::simwild::tight_seal {

bool is_ambient(const std::map<int64_t, std::string>& tag_id_to_name, const int64_t tag)
{
    const auto it = tag_id_to_name.find(tag);
    return it != tag_id_to_name.end() && it->second == "ambient";
}

bool segments_bvh(const std::vector<Vector2d>& vertices, SimpleBVH::BVH& bvh)
{
    if (vertices.empty()) {
        return false;
    }
    MatrixXd V(vertices.size(), 2);
    MatrixXi E(vertices.size() / 2, 2);
    for (size_t i = 0; i < vertices.size(); ++i) {
        V.row(i) = vertices[i];
    }
    for (int i = 0; i < E.rows(); ++i) {
        E.row(i) << 2 * i, 2 * i + 1;
    }
    bvh.init(V, E, 0);
    return true;
}

bool triangles_bvh(const std::vector<Vector3d>& vertices, SimpleBVH::BVH& bvh)
{
    if (vertices.empty()) {
        return false;
    }
    MatrixXd V(vertices.size(), 3);
    MatrixXi F(vertices.size() / 3, 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        V.row(i) = vertices[i];
    }
    for (int i = 0; i < F.rows(); ++i) {
        F.row(i) << 3 * i, 3 * i + 1, 3 * i + 2;
    }
    bvh.init(V, F, 0);
    return true;
}

double bvh_distance(const SimpleBVH::BVH& bvh, const Vector2d& p)
{
    double sq_dist;
    Vector2d nearest_point;
    bvh.nearest_facet(p, nearest_point, sq_dist);
    return std::sqrt(sq_dist);
}

double bvh_distance(const SimpleBVH::BVH& bvh, const Vector3d& p)
{
    double sq_dist;
    Vector3d nearest_point;
    bvh.nearest_facet(p, nearest_point, sq_dist);
    return std::sqrt(sq_dist);
}

namespace {

template <typename Vec>
std::optional<double> sphere_trace_impl(
    const std::function<double(const Vec&)>& f,
    const Vec& a,
    const Vec& dir,
    const double L,
    const double level,
    const double tol,
    const bool from_below)
{
    double t = 0.;
    while (t < L && t >= 0.) {
        const double v = f(a + t * dir);
        if (std::abs(v - level) <= tol) {
            return t;
        }
        t += from_below ? level - v : v - level;
    }
    return std::nullopt;
}

template <typename Vec>
size_t nearest_region_impl(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vec& p,
    const std::vector<size_t>& regions)
{
    size_t best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (const size_t i : regions) {
        const double d = bvh_distance(bvhs[i], p);
        if (d < best_d) {
            best = i;
            best_d = d;
        }
    }
    return best;
}

} // namespace

std::optional<double> sphere_trace(
    const std::function<double(const Vector2d&)>& f,
    const Vector2d& a,
    const Vector2d& dir,
    const double L,
    const double level,
    const double tol,
    const bool from_below)
{
    return sphere_trace_impl(f, a, dir, L, level, tol, from_below);
}

std::optional<double> sphere_trace(
    const std::function<double(const Vector3d&)>& f,
    const Vector3d& a,
    const Vector3d& dir,
    const double L,
    const double level,
    const double tol,
    const bool from_below)
{
    return sphere_trace_impl(f, a, dir, L, level, tol, from_below);
}

size_t nearest_region(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector2d& p,
    const std::vector<size_t>& regions)
{
    return nearest_region_impl(bvhs, p, regions);
}

size_t nearest_region(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector3d& p,
    const std::vector<size_t>& regions)
{
    return nearest_region_impl(bvhs, p, regions);
}

void topology_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const TopologyFillMesh& mesh,
    std::vector<char>& filled,
    const std::function<void()>& after_component)
{
    std::vector<char> pending(filled.size(), 0); // a cell of a component, not labelled yet
    size_t n_cells = 0;
    for (const ConnectedComponent& comp : components) {
        n_cells += comp.cells.size();
        for (const size_t cell : comp.cells) {
            pending[cell] = 1;
        }
    }

    size_t n_left = 0;
    size_t n_left_reached = 0; // of them, sharing a facet with their tag's cells
    for (size_t c = 0; c < components.size(); ++c) {
        const std::vector<size_t>& regions = components_regions[c];
        std::map<size_t, std::vector<size_t>> cells_of_region;
        for (const size_t cell : components[c].cells) {
            cells_of_region[mesh.nearest_region(cell, regions)].push_back(cell);
        }

        for (const auto& [i, cells] : cells_of_region) {
            const int64_t tag = tags[i];
            const std::function<bool(size_t)> in_region = [&](const size_t cell) {
                return !pending[cell] && mesh.tags(cell).count(tag) > 0;
            };
            std::unordered_set<size_t> left(cells.begin(), cells.end());
            std::unordered_set<size_t> queued(cells.begin(), cells.end());
            std::deque<size_t> queue(cells.begin(), cells.end());
            while (!queue.empty()) {
                const size_t cell = queue.front();
                queue.pop_front();
                queued.erase(cell);
                if (!mesh.consistent_topology(cell, in_region)) {
                    continue;
                }
                CellTag& cell_tags = mesh.tags(cell);
                for (const size_t j : regions) {
                    cell_tags.erase(tags[j]);
                }
                cell_tags.insert(tag);
                pending[cell] = 0;
                filled[cell] = 1;
                left.erase(cell);
                for (const size_t nbr : mesh.neighbours(cell)) {
                    if (left.count(nbr) > 0 && queued.insert(nbr).second) {
                        queue.push_back(nbr);
                    }
                }
            }

            if (!left.empty()) {
                size_t n_reached = 0;
                for (const size_t cell : left) {
                    const std::vector<size_t> nbrs = mesh.neighbours(cell);
                    n_reached += std::any_of(nbrs.begin(), nbrs.end(), in_region);
                }
                logger().warn(
                    "tight_seal_geom: fill of component {} ({} cells): {} of the {} cells nearest "
                    "tag {} are left: {} share a facet with it, {} do not",
                    c,
                    components[c].cells.size(),
                    left.size(),
                    cells.size(),
                    tag,
                    n_reached,
                    left.size() - n_reached);
                n_left += left.size();
                n_left_reached += n_reached;
            }
        }
        after_component();
    }

    if (n_left > 0) {
        log_and_throw_error(
            "tight_seal_geom: the fill left {} of the {} cells of the join regions unlabelled: "
            "each fails the topology test with its nearest tag. {} share a facet with that tag, "
            "{} do not",
            n_left,
            n_cells,
            n_left_reached,
            n_left - n_left_reached);
    }
    logger().info(
        "tight_seal_geom: fill of {} components: {} cells labelled, each with its nearest tag, "
        "every tag's topology kept",
        components.size(),
        n_cells);
}

void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles)
{
    for (size_t k = 1; k + 1 < poly.size(); ++k) {
        triangles.push_back(poly[0]);
        triangles.push_back(poly[k]);
        triangles.push_back(poly[k + 1]);
    }
}

} // namespace wmtk::components::simwild::tight_seal


namespace wmtk::components::simwild::tri {

bool SimWildMeshTri::tight_seal_boundary_bvh(
    const std::vector<size_t>& fids,
    const std::function<bool(size_t)>& in_set,
    SimpleBVH::BVH& bvh) const
{
    std::vector<Vector2d> vertices;
    for (const size_t fid : fids) {
        if (!in_set(fid)) {
            continue;
        }
        for (int j = 0; j < 3; ++j) {
            const Tuple edge_tup = tuple_from_edge(fid, j);
            const auto t_opp = edge_tup.switch_face(*this);
            if (t_opp && in_set(t_opp->fid(*this))) {
                continue;
            }
            vertices.push_back(m_vertex_attribute[edge_tup.vid(*this)].m_posf);
            vertices.push_back(m_vertex_attribute[edge_tup.switch_vertex(*this).vid(*this)].m_posf);
        }
    }
    return tight_seal::segments_bvh(vertices, bvh);
}

bool SimWildMeshTri::tight_seal_dilation_surface_bvh(
    const std::vector<size_t>& fids,
    const std::vector<int>& region,
    const std::vector<double>& dist_R_at,
    const std::function<double(const Vector2d&)>& dist_R,
    const double radius,
    const double tol,
    SimpleBVH::BVH& bvh,
    size_t& n_traced,
    size_t& n_failed) const
{
    const auto pos = [&](const size_t vid) -> const Vector2d& {
        return m_vertex_attribute[vid].m_posf;
    };
    const auto in_U = [&](const size_t v) { return dist_R_at[v] >= radius; };
    // the point of edge (v_out, v_in) where d(x, R) reaches d/2, v_in being in U; v_in itself if
    // the trace leaves the edge
    std::map<simplex::Edge, Vector2d> traced;
    const auto crossing = [&](const size_t v_out, const size_t v_in) -> Vector2d {
        const simplex::Edge e(v_out, v_in);
        const auto it = traced.find(e);
        if (it != traced.end()) {
            return it->second;
        }
        const Vector2d a = pos(v_out);
        const double L = (pos(v_in) - a).norm();
        const Vector2d dir = (pos(v_in) - a) / L;
        const auto t = tight_seal::sphere_trace(dist_R, a, dir, L, radius, tol, true);
        if (!t) {
            ++n_failed;
        }
        return traced[e] = t ? Vector2d(a + *t * dir) : pos(v_in);
    };
    std::vector<Vector2d> segments;
    for (const size_t fid : fids) {
        if (region[fid] >= 0) {
            continue;
        }
        const auto vs = oriented_tri_vids(fid);
        std::vector<Vector2d> cut;
        for (int j = 0; j < 3; ++j) {
            const size_t va = vs[j];
            const size_t vb = vs[(j + 1) % 3];
            if (in_U(va) != in_U(vb)) {
                cut.push_back(in_U(va) ? crossing(vb, va) : crossing(va, vb));
            }
        }
        if (cut.size() == 2) {
            segments.push_back(cut[0]);
            segments.push_back(cut[1]);
        }
    }
    n_traced = traced.size();
    return tight_seal::segments_bvh(segments, bvh);
}

size_t SimWildMeshTri::tight_seal_split_tracked(
    const simplex::Edge& e,
    std::unordered_set<size_t>& faces,
    std::vector<char>& filled,
    const std::function<void(size_t, size_t)>& on_child)
{
    const auto [e0, e1] = e.vertices();
    const auto [t, eid] = tuple_from_edge(e.vertices());

    // Each face on the edge gives two children, which keep the face's vertex opposite the
    // edge. Record by that vertex whether the face is in the set, and whether it was filled.
    const std::vector<size_t> parents = get_incident_fids_for_edge(t);
    std::map<size_t, std::pair<bool, char>> parent_state;
    for (const size_t fid : parents) {
        parent_state[simplex_from_face(fid).opposite_vertex(e).id()] = {
            faces.count(fid) > 0,
            filled[fid]};
    }

    std::vector<Tuple> new_tris;
    reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
    if (!split_edge(t, new_tris)) {
        return size_t(-1);
    }

    const size_t v_new = m_last_split_vertex;
    for (const size_t fid : parents) {
        faces.erase(fid);
    }
    filled.resize(tri_capacity(), 0);
    const std::vector<size_t> children = get_one_ring_fids_for_vertex(v_new);
    for (const size_t fid : children) {
        size_t end_vid = -1;
        size_t opp_vid = -1;
        for (const size_t v : oriented_tri_vids(fid)) {
            if (v == e0 || v == e1) {
                end_vid = v;
            } else if (v != v_new) {
                opp_vid = v;
            }
        }
        if (end_vid == size_t(-1) || parent_state.count(opp_vid) == 0) {
            log_and_throw_error("Could not find the parent of a face after a split.");
        }
        const auto [in_set, parent_filled] = parent_state.at(opp_vid);
        filled[fid] = parent_filled;
        if (!in_set) {
            continue; // outside the set, keeps its tags
        }
        faces.insert(fid);
        on_child(fid, end_vid);
    }
    return v_new;
}

size_t SimWildMeshTri::tight_seal_split_face_tracked(
    const size_t fid,
    const Vector2d& p,
    std::unordered_set<size_t>& faces,
    std::vector<char>& filled)
{
    const bool in_set = faces.count(fid) > 0;
    const char parent_filled = filled[fid];
    const Tuple t = tuple_from_tri(fid);
    reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
    m_face_split_position = p;
    std::vector<Tuple> new_tris;
    const bool ok = split_face(t, new_tris);
    m_face_split_position.reset();
    if (!ok) {
        return size_t(-1);
    }
    const size_t v_new = m_last_split_vertex;
    faces.erase(fid);
    filled.resize(tri_capacity(), 0);
    for (const size_t f : get_one_ring_fids_for_vertex(v_new)) {
        filled[f] = parent_filled;
        if (in_set) {
            faces.insert(f);
        }
    }
    return v_new;
}

size_t SimWildMeshTri::tight_seal_cells_on_boundary(const std::vector<size_t>& fids) const
{
    std::vector<char> on_boundary(vert_capacity(), 0);
    for (const Tuple& t : get_faces()) {
        for (int j = 0; j < 3; ++j) {
            const Tuple e = tuple_from_edge(t.fid(*this), j);
            if (!e.switch_face(*this)) {
                on_boundary[e.vid(*this)] = 1;
                on_boundary[e.switch_vertex(*this).vid(*this)] = 1;
            }
        }
    }
    return std::count_if(fids.begin(), fids.end(), [&](const size_t fid) {
        const auto vs = oriented_tri_vids(fid);
        return std::any_of(vs.begin(), vs.end(), [&](const size_t v) { return on_boundary[v]; });
    });
}

void SimWildMeshTri::tight_seal_topology_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    std::vector<char>& filled)
{
    tight_seal::TopologyFillMesh view;
    view.nearest_region = [&](const size_t fid, const std::vector<size_t>& regions) {
        Vector2d centroid = Vector2d::Zero();
        for (const size_t v : oriented_tri_vids(fid)) {
            centroid += m_vertex_attribute[v].m_posf / 3.;
        }
        return tight_seal::nearest_region(region_bvhs, centroid, regions);
    };
    view.consistent_topology = [&](const size_t fid, const std::function<bool(size_t)>& in_region) {
        return wmtk::utils::tri_consistent_topology(*this, fid, in_region);
    };
    view.neighbours = [&](const size_t fid) {
        std::vector<size_t> nbrs;
        for (int j = 0; j < 3; ++j) {
            if (const auto opp = tuple_from_edge(fid, j).switch_face(*this)) {
                nbrs.push_back(opp->fid(*this));
            }
        }
        return nbrs;
    };
    view.tags = [&](const size_t fid) -> CellTag& { return m_face_attribute[fid].tags; };
    tight_seal::topology_fill(components, components_regions, tags, view, filled, [&]() {
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    });
}

void SimWildMeshTri::tight_seal_geom_sanity(
    const std::vector<int64_t>& tags,
    const std::vector<char>& filled) const
{
    const CellTag region_tags(tags.begin(), tags.end());
    std::vector<size_t> filled_ids;
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (is_inverted(fid)) {
            log_and_throw_error("tight_seal_geom sanity check: face {} is inverted", fid);
        }
        if (!filled[fid]) {
            continue;
        }
        filled_ids.push_back(fid);
        const CellTag& tag = m_face_attribute[fid].tags;
        if (std::count_if(tag.begin(), tag.end(), [&](const int64_t x) {
                return region_tags.count(x) > 0;
            }) != 1) {
            log_and_throw_error(
                "tight_seal_geom sanity check: filled face {} has tags {}, not exactly one "
                "of {}",
                fid,
                std::vector<int64_t>(tag.begin(), tag.end()),
                tags);
        }
    }
    std::vector<char> visited(filled.size(), 0);
    size_t n_parts = 0;
    for (const size_t start : filled_ids) {
        if (visited[start]) {
            continue;
        }
        visited[start] = 1;
        std::vector<size_t> part{start};
        CellTag borders;
        for (size_t i = 0; i < part.size(); ++i) { // BFS loop with part growing
            for (int j = 0; j < 3; ++j) {
                const auto t_opp = tuple_from_edge(part[i], j).switch_face(*this);
                if (!t_opp) {
                    continue;
                }
                const size_t nbr = t_opp->fid(*this);
                if (filled[nbr]) {
                    if (!visited[nbr]) {
                        visited[nbr] = 1;
                        part.push_back(nbr);
                    }
                    continue;
                }
                const CellTag& tag = m_face_attribute[nbr].tags;
                if (tag.size() == 1 && region_tags.count(*tag.begin()) > 0) {
                    borders.insert(*tag.begin());
                }
            }
        }
        ++n_parts;
        if (borders.size() < 2) {
            log_and_throw_error(
                "tight_seal_geom sanity check: a filled part of {} faces borders only the "
                "regions {}, so a concave feature of one body was filled",
                part.size(),
                std::vector<int64_t>(borders.begin(), borders.end()));
        }
    }
    logger().info(
        "tight_seal_geom sanity check passed: no face inverted; {} filled faces in {} "
        "parts, each bordering two regions or more",
        filled_ids.size(),
        n_parts);
}

} // namespace wmtk::components::simwild::tri


namespace wmtk::components::simwild {

bool SimWildMesh::tight_seal_boundary_bvh(
    const std::vector<size_t>& tids,
    const std::function<bool(size_t)>& in_set,
    SimpleBVH::BVH& bvh) const
{
    std::vector<Vector3d> vertices;
    for (const size_t tid : tids) {
        if (!in_set(tid)) {
            continue;
        }
        for (int j = 0; j < 4; ++j) {
            const Tuple face_tup = tuple_from_face(tid, j);
            const auto t_opp = face_tup.switch_tetrahedron(*this);
            if (t_opp && in_set(t_opp->tid(*this))) {
                continue;
            }
            vertices.push_back(m_vertex_attribute[face_tup.vid(*this)].m_posf);
            vertices.push_back(m_vertex_attribute[face_tup.switch_vertex(*this).vid(*this)].m_posf);
            vertices.push_back(
                m_vertex_attribute[face_tup.switch_edge(*this).switch_vertex(*this).vid(*this)]
                    .m_posf);
        }
    }
    return tight_seal::triangles_bvh(vertices, bvh);
}

bool SimWildMesh::tight_seal_dilation_surface_bvh(
    const std::vector<size_t>& tids,
    const std::vector<int>& region,
    const std::vector<double>& dist_R_at,
    const std::function<double(const Vector3d&)>& dist_R,
    const double radius,
    const double tol,
    SimpleBVH::BVH& bvh,
    size_t& n_traced,
    size_t& n_failed) const
{
    const auto pos = [&](const size_t vid) -> const Vector3d& {
        return m_vertex_attribute[vid].m_posf;
    };
    const auto in_U = [&](const size_t v) { return dist_R_at[v] >= radius; };
    // the point of edge (v_out, v_in) where d(x, R) reaches d/2, v_in being in U; v_in itself if
    // the trace leaves the edge
    std::map<simplex::Edge, Vector3d> traced;
    const auto crossing = [&](const size_t v_out, const size_t v_in) -> Vector3d {
        const simplex::Edge e(v_out, v_in);
        const auto it = traced.find(e);
        if (it != traced.end()) {
            return it->second;
        }
        const Vector3d a = pos(v_out);
        const double L = (pos(v_in) - a).norm();
        const Vector3d dir = (pos(v_in) - a) / L;
        const auto t = tight_seal::sphere_trace(dist_R, a, dir, L, radius, tol, true);
        if (!t) {
            ++n_failed;
        }
        return traced[e] = t ? Vector3d(a + *t * dir) : pos(v_in);
    };
    std::vector<Vector3d> triangles;
    for (const size_t tid : tids) {
        if (region[tid] >= 0) {
            continue;
        }
        const auto vs = oriented_tet_vids(tid);
        // The cut edges, in order around the cut: with four, the one sharing no vertex with the
        // first is opposite it.
        std::vector<std::array<size_t, 2>> cut;
        for (const auto& [a, b] : tight_seal::tet_edges) {
            if (in_U(vs[a]) != in_U(vs[b])) {
                cut.push_back({{vs[a], vs[b]}});
            }
        }
        if (cut.size() < 3) {
            continue;
        }
        if (cut.size() == 4) {
            for (size_t k = 1; k < 4; ++k) {
                if (cut[k][0] != cut[0][0] && cut[k][0] != cut[0][1] && cut[k][1] != cut[0][0] &&
                    cut[k][1] != cut[0][1]) {
                    std::swap(cut[k], cut[2]);
                    break;
                }
            }
        }
        std::vector<Vector3d> poly;
        for (const auto& [va, vb] : cut) {
            poly.push_back(in_U(va) ? crossing(vb, va) : crossing(va, vb));
        }
        tight_seal::fan_triangles(poly, triangles);
    }
    n_traced = traced.size();
    return tight_seal::triangles_bvh(triangles, bvh);
}

size_t SimWildMesh::tight_seal_split_tracked(
    const simplex::Edge& e,
    std::unordered_set<size_t>& cells,
    std::vector<char>& filled,
    const std::function<void(size_t, size_t)>& on_child,
    std::vector<int>* cell_group)
{
    const auto [e0, e1] = e.vertices();
    const Tuple t = tuple_from_edge(e.vertices());

    // Each tet on the edge gives two children, which keep the tet's edge opposite the split
    // edge. Record by that edge whether the tet is in the set, whether it was filled, and its
    // group.
    const std::vector<size_t> parents = get_incident_tids_for_edge(t);
    std::map<simplex::Edge, std::tuple<bool, char, int>> parent_state;
    for (const size_t tid : parents) {
        parent_state[simplex_from_tet(tid).opposite_edge(e)] = {
            cells.count(tid) > 0,
            filled[tid],
            cell_group ? (*cell_group)[tid] : -1};
    }

    std::vector<Tuple> new_edges;
    reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
    if (!split_edge(t, new_edges)) {
        return size_t(-1);
    }

    const size_t v_new = split_tag_cache.local().v_new;
    for (const size_t tid : parents) {
        cells.erase(tid);
    }
    filled.resize(tet_capacity(), 0);
    if (cell_group) {
        cell_group->resize(tet_capacity(), -1);
    }
    const std::vector<size_t> children = get_one_ring_tids_for_vertex(v_new);
    for (const size_t tid : children) {
        size_t end_vid = -1;
        std::vector<size_t> opp;
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v == e0 || v == e1) {
                end_vid = v;
            } else if (v != v_new) {
                opp.push_back(v);
            }
        }
        if (end_vid == size_t(-1) || opp.size() != 2) {
            log_and_throw_error("Could not find the parent of a tet after a split.");
        }
        const auto it = parent_state.find(simplex::Edge(opp[0], opp[1]));
        if (it == parent_state.end()) {
            log_and_throw_error("Could not find the parent of a tet after a split.");
        }
        const auto [in_set, parent_filled, parent_group] = it->second;
        filled[tid] = parent_filled;
        if (cell_group) {
            (*cell_group)[tid] = parent_group;
        }
        if (!in_set) {
            continue; // outside the set, keeps its tags
        }
        cells.insert(tid);
        on_child(tid, end_vid);
    }
    return v_new;
}

size_t SimWildMesh::tight_seal_split_face_tracked(
    const std::array<size_t, 3>& f,
    const Vector3d& p,
    std::unordered_set<size_t>& cells,
    std::vector<char>& filled,
    std::vector<int>* cell_group)
{
    std::array<size_t, 3> sorted = f;
    std::sort(sorted.begin(), sorted.end());
    const Tuple t = std::get<0>(tuple_from_face(sorted));

    // The tets on the face give three children each, which keep the tet's corner off the face.
    // Record by that corner whether the tet is in the set, whether it was filled, and its group.
    std::vector<size_t> parents = {t.tid(*this)};
    if (const auto opp = t.switch_tetrahedron(*this)) {
        parents.push_back(opp->tid(*this));
    }
    const auto off_face = [&](const size_t tid) {
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v != f[0] && v != f[1] && v != f[2]) {
                return v;
            }
        }
        return size_t(-1);
    };
    std::map<size_t, std::tuple<bool, char, int>> parent_state;
    for (const size_t tid : parents) {
        parent_state[off_face(tid)] = {
            cells.count(tid) > 0,
            filled[tid],
            cell_group ? (*cell_group)[tid] : -1};
    }

    reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
    m_face_split_position = p;
    std::vector<Tuple> new_tets;
    const bool ok = split_face(t, new_tets);
    m_face_split_position.reset();
    if (!ok) {
        return size_t(-1);
    }
    const size_t v_new = m_last_face_split_vertex;
    for (const size_t tid : parents) {
        cells.erase(tid);
    }
    filled.resize(tet_capacity(), 0);
    if (cell_group) {
        cell_group->resize(tet_capacity(), -1);
    }
    for (const size_t tid : get_one_ring_tids_for_vertex(v_new)) {
        size_t off = size_t(-1);
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v != f[0] && v != f[1] && v != f[2] && v != v_new) {
                off = v;
            }
        }
        const auto it = parent_state.find(off);
        if (it == parent_state.end()) {
            log_and_throw_error("Could not find the parent of a tet after a face split.");
        }
        const auto [in_set, parent_filled, parent_group] = it->second;
        filled[tid] = parent_filled;
        if (cell_group) {
            (*cell_group)[tid] = parent_group;
        }
        if (in_set) {
            cells.insert(tid);
        }
    }
    return v_new;
}

size_t SimWildMesh::tight_seal_cells_on_boundary(const std::vector<size_t>& tids) const
{
    std::vector<char> on_boundary(vert_capacity(), 0);
    for (const Tuple& t : get_tets()) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(t.tid(*this), j);
            if (!f.switch_tetrahedron(*this)) {
                for (const size_t v : get_face_vids(f)) {
                    on_boundary[v] = 1;
                }
            }
        }
    }
    return std::count_if(tids.begin(), tids.end(), [&](const size_t tid) {
        const auto vs = oriented_tet_vids(tid);
        return std::any_of(vs.begin(), vs.end(), [&](const size_t v) { return on_boundary[v]; });
    });
}

void SimWildMesh::tight_seal_topology_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    std::vector<char>& filled)
{
    tight_seal::TopologyFillMesh view;
    view.nearest_region = [&](const size_t tid, const std::vector<size_t>& regions) {
        Vector3d centroid = Vector3d::Zero();
        for (const size_t v : oriented_tet_vids(tid)) {
            centroid += m_vertex_attribute[v].m_posf / 4.;
        }
        return tight_seal::nearest_region(region_bvhs, centroid, regions);
    };
    view.consistent_topology = [&](const size_t tid, const std::function<bool(size_t)>& in_region) {
        return wmtk::utils::tet_consistent_topology(*this, tid, in_region);
    };
    view.neighbours = [&](const size_t tid) {
        std::vector<size_t> nbrs;
        for (int j = 0; j < 4; ++j) {
            if (const auto opp = tuple_from_face(tid, j).switch_tetrahedron(*this)) {
                nbrs.push_back(opp->tid(*this));
            }
        }
        return nbrs;
    };
    view.tags = [&](const size_t tid) -> CellTag& { return m_tet_attribute[tid].tags; };
    tight_seal::topology_fill(components, components_regions, tags, view, filled, [&]() {
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    });
}

void SimWildMesh::tight_seal_geom_sanity(
    const std::vector<int64_t>& tags,
    const std::vector<char>& filled) const
{
    const CellTag region_tags(tags.begin(), tags.end());
    std::vector<size_t> filled_ids;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (is_inverted(t)) {
            log_and_throw_error("tight_seal_geom sanity check: tet {} is inverted", tid);
        }
        if (!filled[tid]) {
            continue;
        }
        filled_ids.push_back(tid);
        const CellTag& tag = m_tet_attribute[tid].tags;
        if (std::count_if(tag.begin(), tag.end(), [&](const int64_t x) {
                return region_tags.count(x) > 0;
            }) != 1) {
            log_and_throw_error(
                "tight_seal_geom sanity check: filled tet {} has tags {}, not exactly one "
                "of {}",
                tid,
                std::vector<int64_t>(tag.begin(), tag.end()),
                tags);
        }
    }
    std::vector<char> visited(filled.size(), 0);
    size_t n_parts = 0;
    for (const size_t start : filled_ids) {
        if (visited[start]) {
            continue;
        }
        visited[start] = 1;
        std::vector<size_t> part{start};
        CellTag borders;
        for (size_t i = 0; i < part.size(); ++i) { // BFS loop with part growing
            for (int j = 0; j < 4; ++j) {
                const auto t_opp = tuple_from_face(part[i], j).switch_tetrahedron(*this);
                if (!t_opp) {
                    continue;
                }
                const size_t nbr = t_opp->tid(*this);
                if (filled[nbr]) {
                    if (!visited[nbr]) {
                        visited[nbr] = 1;
                        part.push_back(nbr);
                    }
                    continue;
                }
                const CellTag& tag = m_tet_attribute[nbr].tags;
                if (tag.size() == 1 && region_tags.count(*tag.begin()) > 0) {
                    borders.insert(*tag.begin());
                }
            }
        }
        ++n_parts;
        if (borders.size() < 2) {
            log_and_throw_error(
                "tight_seal_geom sanity check: a filled part of {} tets borders only the "
                "regions {}, so a concave feature of one body was filled",
                part.size(),
                std::vector<int64_t>(borders.begin(), borders.end()));
        }
    }
    logger().info(
        "tight_seal_geom sanity check passed: no tet inverted; {} filled tets in {} "
        "parts, each bordering two regions or more",
        filled_ids.size(),
        n_parts);
}

} // namespace wmtk::components::simwild
