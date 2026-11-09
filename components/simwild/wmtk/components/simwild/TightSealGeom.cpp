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
#include <optional>
#include <set>
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
std::vector<size_t> regions_by_distance_impl(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vec& p,
    const std::vector<size_t>& regions)
{
    std::vector<std::pair<double, size_t>> ranked;
    ranked.reserve(regions.size());
    for (const size_t i : regions) {
        ranked.emplace_back(bvh_distance(bvhs[i], p), i);
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    std::vector<size_t> out;
    out.reserve(ranked.size());
    for (const auto& r : ranked) {
        out.push_back(r.second);
    }
    return out;
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

std::vector<size_t> regions_by_distance(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector2d& p,
    const std::vector<size_t>& regions)
{
    return regions_by_distance_impl(bvhs, p, regions);
}

std::vector<size_t> regions_by_distance(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector3d& p,
    const std::vector<size_t>& regions)
{
    return regions_by_distance_impl(bvhs, p, regions);
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
    size_t n_farther = 0; // cells labelled with other than their nearest region
    for (size_t c = 0; c < components.size(); ++c) {
        const std::vector<size_t>& regions = components_regions[c];
        const auto in_region = [&](const int64_t tag) {
            return std::function<bool(size_t)>([&pending, &mesh, tag](const size_t cell) {
                return !pending[cell] && mesh.tags(cell).count(tag) > 0;
            });
        };
        const auto label = [&](const size_t cell, const int64_t tag) {
            CellTag& cell_tags = mesh.tags(cell);
            for (const size_t j : regions) {
                cell_tags.erase(tags[j]);
            }
            cell_tags.insert(tag);
            pending[cell] = 0;
            filled[cell] = 1;
        };

        std::map<size_t, std::vector<size_t>> cells_of_region;
        for (const size_t cell : components[c].cells) {
            cells_of_region[mesh.regions_by_distance(cell, regions).front()].push_back(cell);
        }

        // Each region grows over the cells nearest it; those it cannot add wait in `refused`.
        std::vector<size_t> refused;
        for (const auto& [i, cells] : cells_of_region) {
            const int64_t tag = tags[i];
            const std::function<bool(size_t)> in_tag = in_region(tag);
            std::unordered_set<size_t> left(cells.begin(), cells.end());
            std::unordered_set<size_t> queued(cells.begin(), cells.end());
            std::deque<size_t> queue(cells.begin(), cells.end());
            while (!queue.empty()) {
                const size_t cell = queue.front();
                queue.pop_front();
                queued.erase(cell);
                if (!mesh.consistent_topology(cell, in_tag)) {
                    continue;
                }
                label(cell, tag);
                left.erase(cell);
                for (const size_t nbr : mesh.neighbours(cell)) {
                    if (left.count(nbr) > 0 && queued.insert(nbr).second) {
                        queue.push_back(nbr);
                    }
                }
            }
            for (const size_t cell : cells) {
                if (left.count(cell) > 0) {
                    refused.push_back(cell);
                }
            }
        }

        // With every region tried, each refused cell takes the nearest region that passes the
        // test. Its own nearest may pass by now, and a cell may pass only once a neighbour has
        // been labelled, so the cells are gone over until a pass labels none.
        bool labelled = !refused.empty();
        while (labelled) {
            labelled = false;
            std::vector<size_t> still_refused;
            for (const size_t cell : refused) {
                const std::vector<size_t> ranked = mesh.regions_by_distance(cell, regions);
                const auto passes = std::find_if(ranked.begin(), ranked.end(), [&](const size_t i) {
                    return mesh.consistent_topology(cell, in_region(tags[i]));
                });
                if (passes == ranked.end()) {
                    still_refused.push_back(cell);
                    continue;
                }
                label(cell, tags[*passes]);
                n_farther += passes != ranked.begin();
                labelled = true;
            }
            refused = std::move(still_refused);
        }

        if (!refused.empty()) {
            logger().warn(
                "tight_seal_geom: fill of component {} ({} cells): {} cells are left, passing the "
                "topology test with none of its {} tags",
                c,
                components[c].cells.size(),
                refused.size(),
                regions.size());
            n_left += refused.size();
        }
        after_component();
    }

    if (n_left > 0) {
        log_and_throw_error(
            "tight_seal_geom: the fill left {} of the {} cells of the join regions unlabelled: "
            "each fails the topology test with every tag its part borders",
            n_left,
            n_cells);
    }
    logger().info(
        "tight_seal_geom: fill of {} components: {} cells labelled, {} of them with other than "
        "their nearest tag, every tag's topology kept",
        components.size(),
        n_cells,
        n_farther);
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

    const size_t v_new = m_last_split_vertex.local();
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
    const size_t v_new = m_last_split_vertex.local();
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
    view.regions_by_distance = [&](const size_t fid, const std::vector<size_t>& regions) {
        Vector2d centroid = Vector2d::Zero();
        for (const size_t v : oriented_tri_vids(fid)) {
            centroid += m_vertex_attribute[v].m_posf / 3.;
        }
        return tight_seal::regions_by_distance(region_bvhs, centroid, regions);
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

void SimWildMeshTri::tight_seal_split_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    std::vector<char>& filled)
{
    const auto pos = [&](const size_t v) -> const Vector2d& {
        return m_vertex_attribute[v].m_posf;
    };

    // Each component's queues, from its own faces before any split of any component: its faces
    // with three distinct vertex labels, its edges with two. A vertex's label is the nearest of the
    // component's regions (the first in their order on a tie).
    struct Queues
    {
        std::map<size_t, size_t> label;
        std::set<std::array<size_t, 3>> faces;
        std::set<std::array<size_t, 2>> edges;
    };
    std::vector<Queues> queues(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        Queues& q = queues[c];
        const auto label_of = [&](const size_t v) {
            auto it = q.label.find(v);
            if (it == q.label.end()) {
                const size_t nearest =
                    tight_seal::regions_by_distance(region_bvhs, pos(v), components_regions[c])
                        .front();
                it = q.label.emplace(v, nearest).first;
            }
            return it->second;
        };
        for (const size_t fid : components[c].cells) {
            std::array<size_t, 3> vs = oriented_tri_vids(fid);
            std::sort(vs.begin(), vs.end());
            const std::set<size_t> found = {label_of(vs[0]), label_of(vs[1]), label_of(vs[2])};
            if (found.size() == 3) {
                q.faces.insert(vs);
            }
            for (int j = 0; j < 3; ++j) {
                for (int k = j + 1; k < 3; ++k) {
                    if (label_of(vs[j]) != label_of(vs[k])) {
                        q.edges.insert({{vs[j], vs[k]}});
                    }
                }
            }
        }
    }

    // The splits, component by component, each queue in turn: the faces at their centroid, the
    // edges where the distances to their two ends' labels are equal (the bisection of
    // split_adjust_position on m_voronoi_split_fn). Components meet at vertices only, so each
    // component's faces are tracked in a set of its own.
    std::vector<std::unordered_set<size_t>> comp_faces(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        comp_faces[c].insert(components[c].cells.begin(), components[c].cells.end());
    }
    Vector2d lo = Vector2d::Constant(std::numeric_limits<double>::infinity());
    Vector2d hi = -lo;
    for (const Tuple& t : get_vertices()) {
        lo = lo.cwiseMin(pos(t.vid(*this)));
        hi = hi.cwiseMax(pos(t.vid(*this)));
    }
    const double tie = 1e-10 * (hi - lo).norm(); // distances this close count as equal

    size_t n_faces = 0, n_edges = 0, n_gone = 0, n_tied = 0, n_refused = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const Queues& q = queues[c];
        for (const auto& vs : q.faces) {
            std::optional<size_t> fid;
            for (const size_t f : get_one_ring_fids_for_vertex(vs[0])) {
                std::array<size_t, 3> ws = oriented_tri_vids(f);
                std::sort(ws.begin(), ws.end());
                if (ws == vs) {
                    fid = f;
                }
            }
            if (!fid) {
                ++n_gone;
                continue;
            }
            const Vector2d centroid = (pos(vs[0]) + pos(vs[1]) + pos(vs[2])) / 3.;
            if (tight_seal_split_face_tracked(*fid, centroid, comp_faces[c], filled) ==
                size_t(-1)) {
                ++n_refused;
            } else {
                ++n_faces;
            }
        }
        for (const auto& [a, b] : q.edges) {
            if (!try_tuple_from_edge({{a, b}})) {
                ++n_gone;
                continue;
            }
            const size_t i = q.label.at(a);
            const size_t j = q.label.at(b);
            const std::function<double(const Vector2d&)> difference = [&, i, j](const Vector2d& p) {
                return tight_seal::bvh_distance(region_bvhs[i], p) -
                       tight_seal::bvh_distance(region_bvhs[j], p);
            };
            // a's label is nearest at a and b's at b; an end as near the other label puts the
            // equality there, and the edge is not split
            if (difference(pos(a)) > -tie || difference(pos(b)) < tie) {
                ++n_tied;
                continue;
            }
            m_voronoi_split_fn = difference;
            const size_t v = tight_seal_split_tracked(
                simplex::Edge(a, b),
                comp_faces[c],
                filled,
                [](const size_t, const size_t) {});
            m_voronoi_split_fn = nullptr;
            if (v == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_edges;
            }
        }
    }
    logger().info(
        "tight_seal_geom: split_fill split {} faces and {} edges of {} components; {} queued "
        "elements were gone, {} edges have an end as near both labels and are not split, {} "
        "splits were refused",
        n_faces,
        n_edges,
        components.size(),
        n_gone,
        n_tied,
        n_refused);

    std::vector<ConnectedComponent> split(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        split[c].cells.assign(comp_faces[c].begin(), comp_faces[c].end());
        std::sort(split[c].cells.begin(), split[c].cells.end());
    }
    tight_seal_topology_fill(split, components_regions, tags, region_bvhs, filled);
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
    view.regions_by_distance = [&](const size_t tid, const std::vector<size_t>& regions) {
        Vector3d centroid = Vector3d::Zero();
        for (const size_t v : oriented_tet_vids(tid)) {
            centroid += m_vertex_attribute[v].m_posf / 4.;
        }
        return tight_seal::regions_by_distance(region_bvhs, centroid, regions);
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

size_t SimWildMesh::tight_seal_split_tet_tracked(
    const size_t tid,
    std::unordered_set<size_t>& cells,
    std::vector<char>& filled,
    std::vector<int>* cell_group)
{
    const bool in_set = cells.count(tid) > 0;
    const char parent_filled = filled[tid];
    const int parent_group = cell_group ? (*cell_group)[tid] : -1;
    const std::array<size_t, 4> parent = oriented_tet_vids(tid);
    const Tuple t = tuple_from_tet(tid);
    reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
    std::vector<Tuple> new_tets;
    if (!split_tet(t, new_tets)) {
        return size_t(-1);
    }
    // the split keeps tid as one of the children, with the new vertex in place of a corner
    size_t v_new = size_t(-1);
    for (const size_t v : oriented_tet_vids(tid)) {
        if (std::find(parent.begin(), parent.end(), v) == parent.end()) {
            v_new = v;
        }
    }
    if (v_new == size_t(-1)) {
        log_and_throw_error("Could not find the new vertex of a tet split.");
    }
    cells.erase(tid);
    filled.resize(tet_capacity(), 0);
    if (cell_group) {
        cell_group->resize(tet_capacity(), -1);
    }
    for (const size_t child : get_one_ring_tids_for_vertex(v_new)) {
        filled[child] = parent_filled;
        if (cell_group) {
            (*cell_group)[child] = parent_group;
        }
        if (in_set) {
            cells.insert(child);
        }
    }
    return v_new;
}

void SimWildMesh::tight_seal_split_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    std::vector<char>& filled)
{
    const auto pos = [&](const size_t v) -> const Vector3d& {
        return m_vertex_attribute[v].m_posf;
    };

    // Each component's queues, from its own tets before any split of any component: its tets
    // with three or four distinct vertex labels, its faces with three, its edges with two. A
    // vertex's label is the nearest of the component's regions (the first in their order on a tie).
    struct Queues
    {
        std::map<size_t, size_t> label;
        std::set<std::array<size_t, 4>> tets;
        std::set<std::array<size_t, 3>> faces;
        std::set<std::array<size_t, 2>> edges;
    };
    std::vector<Queues> queues(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        Queues& q = queues[c];
        const auto label_of = [&](const size_t v) {
            auto it = q.label.find(v);
            if (it == q.label.end()) {
                const size_t nearest =
                    tight_seal::regions_by_distance(region_bvhs, pos(v), components_regions[c])
                        .front();
                it = q.label.emplace(v, nearest).first;
            }
            return it->second;
        };
        const auto n_labels = [&](const auto& vs) {
            std::set<size_t> found;
            for (const size_t v : vs) {
                found.insert(label_of(v));
            }
            return found.size();
        };
        for (const size_t tid : components[c].cells) {
            std::array<size_t, 4> vs = oriented_tet_vids(tid);
            std::sort(vs.begin(), vs.end());
            if (n_labels(vs) >= 3) {
                q.tets.insert(vs);
            }
            for (int skip = 0; skip < 4; ++skip) {
                std::array<size_t, 3> f;
                for (int j = 0, k = 0; j < 4; ++j) {
                    if (j != skip) {
                        f[k++] = vs[j];
                    }
                }
                if (n_labels(f) == 3) {
                    q.faces.insert(f);
                }
            }
            for (const auto& [a, b] : tight_seal::tet_edges) {
                if (label_of(vs[a]) != label_of(vs[b])) {
                    q.edges.insert({{vs[a], vs[b]}});
                }
            }
        }
    }

    // The splits, component by component, each queue in turn: the tets at their centroid, the
    // faces at theirs, the edges where the distances to their two ends' labels are equal (the
    // bisection of split_adjust_position on m_voronoi_split_fn). Every component's tets are
    // tracked: two components can share an edge, and splitting it for one splits the other's tets,
    // so an element of a later queue may be gone.
    std::vector<int> cell_comp(tet_capacity(), -1);
    std::unordered_set<size_t> cells;
    for (size_t c = 0; c < components.size(); ++c) {
        for (const size_t tid : components[c].cells) {
            cell_comp[tid] = int(c);
            cells.insert(tid);
        }
    }
    Vector3d lo = Vector3d::Constant(std::numeric_limits<double>::infinity());
    Vector3d hi = -lo;
    for (const Tuple& t : get_vertices()) {
        lo = lo.cwiseMin(pos(t.vid(*this)));
        hi = hi.cwiseMax(pos(t.vid(*this)));
    }
    const double tie = 1e-10 * (hi - lo).norm(); // distances this close count as equal

    const auto find_tet = [&](const std::array<size_t, 4>& vs) -> std::optional<size_t> {
        for (const size_t tid : get_one_ring_tids_for_vertex(vs[0])) {
            std::array<size_t, 4> ws = oriented_tet_vids(tid);
            std::sort(ws.begin(), ws.end());
            if (ws == vs) {
                return tid;
            }
        }
        return std::nullopt;
    };
    size_t n_tets = 0, n_faces = 0, n_edges = 0, n_gone = 0, n_tied = 0, n_refused = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const Queues& q = queues[c];
        for (const auto& vs : q.tets) {
            const std::optional<size_t> tid = find_tet(vs);
            if (!tid) {
                ++n_gone;
            } else if (
                tight_seal_split_tet_tracked(*tid, cells, filled, &cell_comp) == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_tets;
            }
        }
        for (const auto& f : q.faces) {
            if (!try_tuple_from_face(f)) {
                ++n_gone;
                continue;
            }
            const Vector3d centroid = (pos(f[0]) + pos(f[1]) + pos(f[2])) / 3.;
            if (tight_seal_split_face_tracked(f, centroid, cells, filled, &cell_comp) ==
                size_t(-1)) {
                ++n_refused;
            } else {
                ++n_faces;
            }
        }
        for (const auto& [a, b] : q.edges) {
            if (get_incident_tids_for_edge(a, b).empty()) {
                ++n_gone;
                continue;
            }
            const size_t i = q.label.at(a);
            const size_t j = q.label.at(b);
            const std::function<double(const Vector3d&)> difference = [&, i, j](const Vector3d& p) {
                return tight_seal::bvh_distance(region_bvhs[i], p) -
                       tight_seal::bvh_distance(region_bvhs[j], p);
            };
            // a's label is nearest at a and b's at b; an end as near the other label puts the
            // equality there, and the edge is not split
            if (difference(pos(a)) > -tie || difference(pos(b)) < tie) {
                ++n_tied;
                continue;
            }
            m_voronoi_split_fn = difference;
            const size_t v = tight_seal_split_tracked(
                simplex::Edge(a, b),
                cells,
                filled,
                [](const size_t, const size_t) {},
                &cell_comp);
            m_voronoi_split_fn = nullptr;
            if (v == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_edges;
            }
        }
    }
    logger().info(
        "tight_seal_geom: split_fill split {} tets, {} faces and {} edges of {} components; {} "
        "queued elements were gone (an edge shared with a component split before), {} edges have "
        "an end as near both labels and are not split, {} splits were refused",
        n_tets,
        n_faces,
        n_edges,
        components.size(),
        n_gone,
        n_tied,
        n_refused);

    std::vector<ConnectedComponent> split(components.size());
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (tid < cell_comp.size() && cell_comp[tid] >= 0) {
            split[size_t(cell_comp[tid])].cells.push_back(tid);
        }
    }
    tight_seal_topology_fill(split, components_regions, tags, region_bvhs, filled);
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
