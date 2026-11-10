#include "TightSealGeom.hpp"

#include "SimWildMesh.h"
#include "SimWildMeshTri.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
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

std::pair<size_t, int> split_fill_label(
    const CornerLabels& corners,
    const std::function<const std::vector<size_t>&(size_t)>& ties)
{
    if (corners.empty()) {
        log_and_throw_error("tight_seal_geom: a cell of split_fill has no labelled corner");
    }
    const size_t first = corners.front().second;
    if (std::all_of(corners.begin(), corners.end(), [&](const auto& c) {
            return c.second == first;
        })) {
        return {first, 0};
    }
    std::vector<size_t> shared = ties(corners.front().first);
    for (size_t k = 1; k < corners.size() && !shared.empty(); ++k) {
        const std::vector<size_t>& other = ties(corners[k].first);
        std::vector<size_t> both;
        std::set_intersection(
            shared.begin(),
            shared.end(),
            other.begin(),
            other.end(),
            std::back_inserter(both));
        shared = std::move(both);
    }
    if (shared.size() == 1) {
        return {shared.front(), 1};
    }
    return {shared.empty() ? first : shared.front(), 2};
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
    const std::function<void(size_t, size_t)>& on_child,
    std::vector<CornerLabels>* corner_labels)
{
    const auto [e0, e1] = e.vertices();
    const auto [t, eid] = tuple_from_edge(e.vertices());

    // Each face on the edge gives two children, which keep the face's vertex opposite the
    // edge. Record by that vertex whether the face is in the set, whether it was filled, and its
    // corner labels.
    const std::vector<size_t> parents = get_incident_fids_for_edge(t);
    std::map<size_t, std::tuple<bool, char, CornerLabels>> parent_state;
    for (const size_t fid : parents) {
        parent_state[simplex_from_face(fid).opposite_vertex(e).id()] = {
            faces.count(fid) > 0,
            filled[fid],
            corner_labels && fid < corner_labels->size() ? (*corner_labels)[fid] : CornerLabels{}};
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
    if (corner_labels) {
        corner_labels->resize(tri_capacity());
    }
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
        const auto& [in_set, parent_filled, parent_labels] = parent_state.at(opp_vid);
        filled[fid] = parent_filled;
        if (corner_labels) {
            (*corner_labels)[fid] =
                tight_seal::kept_corner_labels(parent_labels, oriented_tri_vids(fid));
        }
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
    std::vector<char>& filled,
    std::vector<CornerLabels>* corner_labels)
{
    const bool in_set = faces.count(fid) > 0;
    const char parent_filled = filled[fid];
    const CornerLabels parent_labels =
        corner_labels && fid < corner_labels->size() ? (*corner_labels)[fid] : CornerLabels{};
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
    if (corner_labels) {
        corner_labels->resize(tri_capacity());
    }
    for (const size_t f : get_one_ring_fids_for_vertex(v_new)) {
        filled[f] = parent_filled;
        if (corner_labels) {
            (*corner_labels)[f] =
                tight_seal::kept_corner_labels(parent_labels, oriented_tri_vids(f));
        }
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
    Vector2d lo = Vector2d::Constant(std::numeric_limits<double>::infinity());
    Vector2d hi = -lo;
    for (const Tuple& t : get_vertices()) {
        lo = lo.cwiseMin(pos(t.vid(*this)));
        hi = hi.cwiseMax(pos(t.vid(*this)));
    }
    const double tie = 1e-10 * (hi - lo).norm(); // distances this close count as equal

    // A vertex's nearest region of a component (the first on an exact tie) and its tie set: the
    // regions within `tie` of the nearest, ascending.
    struct Nearest
    {
        size_t region;
        std::vector<size_t> ties;
    };
    std::vector<std::map<size_t, Nearest>> nearest(components.size());
    const auto nearest_of = [&](const size_t c, const size_t v) -> const Nearest& {
        auto it = nearest[c].find(v);
        if (it == nearest[c].end()) {
            const std::vector<size_t>& regions = components_regions[c];
            std::vector<double> d(regions.size());
            for (size_t k = 0; k < regions.size(); ++k) {
                d[k] = tight_seal::bvh_distance(region_bvhs[regions[k]], pos(v));
            }
            const size_t best = std::min_element(d.begin(), d.end()) - d.begin();
            Nearest n{regions[best], {}};
            for (size_t k = 0; k < regions.size(); ++k) {
                if (d[k] - d[best] < tie) {
                    n.ties.push_back(regions[k]);
                }
            }
            std::sort(n.ties.begin(), n.ties.end());
            it = nearest[c].emplace(v, std::move(n)).first;
        }
        return it->second;
    };

    // Each face's own labels of its corners: its corner's nearest region, but the ends of an edge
    // it shares with a face of one of the component's regions (a face with that region's tag
    // alone) take that region. The queues come from these, all before any split: the faces with
    // three labels, the edges whose ends differ in a face holding them, with the label pairs asked.
    std::vector<CornerLabels> corner_labels(tri_capacity());
    struct Queues
    {
        std::set<std::array<size_t, 3>> faces;
        std::map<std::array<size_t, 2>, std::set<std::pair<size_t, size_t>>> edges;
    };
    std::vector<Queues> queues(components.size());
    size_t n_on_region = 0, n_on_two = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        std::map<int64_t, size_t> region_of_tag;
        for (const size_t i : components_regions[c]) {
            region_of_tag[tags[i]] = i;
        }
        Queues& q = queues[c];
        for (const size_t fid : components[c].cells) {
            std::array<size_t, 3> vs = oriented_tri_vids(fid);
            std::sort(vs.begin(), vs.end());
            std::map<size_t, size_t> label;
            for (const size_t v : vs) {
                label[v] = nearest_of(c, v).region;
            }
            int on = 0;
            for (int j = 0; j < 3; ++j) {
                const Tuple e = tuple_from_edge(fid, j);
                const auto opp = e.switch_face(*this);
                if (!opp) {
                    continue;
                }
                const CellTag& beyond = m_face_attribute[opp->fid(*this)].tags;
                if (beyond.size() != 1) {
                    continue;
                }
                const auto r = region_of_tag.find(*beyond.begin());
                if (r == region_of_tag.end()) {
                    continue;
                }
                if (on++ == 0) {
                    label[e.vid(*this)] = r->second;
                    label[e.switch_vertex(*this).vid(*this)] = r->second;
                }
            }
            n_on_region += on > 0;
            n_on_two += on > 1;
            CornerLabels& own = corner_labels[fid];
            own.assign(label.begin(), label.end());
            if (label[vs[0]] != label[vs[1]] && label[vs[0]] != label[vs[2]] &&
                label[vs[1]] != label[vs[2]]) {
                q.faces.insert(vs);
            }
            for (int j = 0; j < 3; ++j) {
                for (int k = j + 1; k < 3; ++k) {
                    if (label[vs[j]] != label[vs[k]]) {
                        q.edges[{{vs[j], vs[k]}}].insert({label[vs[j]], label[vs[k]]});
                    }
                }
            }
        }
    }
    if (n_on_two > 0) {
        logger().warn(
            "tight_seal_geom: split_fill: {} faces share edges with two regions' faces or more "
            "(the "
            "regions are not simplicially embedded there); the first edge's region labels its ends",
            n_on_two);
    }

    // The splits, component by component, each queue in turn: the faces at their centroid, the
    // edges where the distances to a pair of labels asked for are equal (the bisection of
    // split_adjust_position on m_voronoi_split_fn). An edge is not split if that falls on an end
    // for every pair asked; split_fill_label settles it. Components meet at vertices only, so
    // each component's faces are tracked in a set of its own.
    std::vector<std::unordered_set<size_t>> comp_faces(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        comp_faces[c].insert(components[c].cells.begin(), components[c].cells.end());
    }
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
            if (tight_seal_split_face_tracked(
                    *fid,
                    centroid,
                    comp_faces[c],
                    filled,
                    &corner_labels) == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_faces;
            }
        }
        for (const auto& [ends, pairs] : q.edges) {
            const size_t a = ends[0];
            const size_t b = ends[1];
            if (!try_tuple_from_edge({{a, b}})) {
                ++n_gone;
                continue;
            }
            std::function<double(const Vector2d&)> difference;
            for (const auto& pair : pairs) {
                const size_t i = pair.first;
                const size_t j = pair.second;
                std::function<double(const Vector2d&)> d = [&, i, j](const Vector2d& p) {
                    return tight_seal::bvh_distance(region_bvhs[i], p) -
                           tight_seal::bvh_distance(region_bvhs[j], p);
                };
                if (d(pos(a)) <= -tie && d(pos(b)) >= tie) {
                    difference = d;
                    break;
                }
            }
            if (!difference) {
                ++n_tied;
                continue;
            }
            m_voronoi_split_fn = difference;
            const size_t v = tight_seal_split_tracked(
                simplex::Edge(a, b),
                comp_faces[c],
                filled,
                [](const size_t, const size_t) {},
                &corner_labels);
            m_voronoi_split_fn = nullptr;
            if (v == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_edges;
            }
        }
    }
    logger().info(
        "tight_seal_geom: split_fill split {} faces and {} edges of {} components ({} faces with "
        "an edge on a region); {} queued elements were gone, {} edges were not split because the "
        "equality fell on an end, {} splits were refused",
        n_faces,
        n_edges,
        components.size(),
        n_on_region,
        n_gone,
        n_tied,
        n_refused);

    // Each face of the components takes a region from its corners' own labels.
    size_t n_agree = 0, n_ties = 0, n_arbitrary = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        for (const size_t fid : comp_faces[c]) {
            const std::pair<size_t, int> chosen = tight_seal::split_fill_label(
                corner_labels[fid],
                [&](const size_t v) -> const std::vector<size_t>& {
                    return nearest_of(c, v).ties;
                });
            CellTag& cell_tags = m_face_attribute[fid].tags;
            for (const size_t j : components_regions[c]) {
                cell_tags.erase(tags[j]);
            }
            cell_tags.insert(tags[chosen.first]);
            filled[fid] = 1;
            ++(chosen.second == 0 ? n_agree : chosen.second == 1 ? n_ties : n_arbitrary);
        }
    }
    logger().info(
        "tight_seal_geom: split_fill labelled {} faces: {} by their corners' own labels, {} by the "
        "one region their corners' tie sets share, {} arbitrarily (an exact tie between regions)",
        n_agree + n_ties + n_arbitrary,
        n_agree,
        n_ties,
        n_arbitrary);
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
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
    std::vector<int>* cell_group,
    std::vector<CornerLabels>* corner_labels)
{
    const auto [e0, e1] = e.vertices();
    const Tuple t = tuple_from_edge(e.vertices());

    // Each tet on the edge gives two children, which keep the tet's edge opposite the split
    // edge. Record by that edge whether the tet is in the set, whether it was filled, its group
    // and its corner labels.
    const std::vector<size_t> parents = get_incident_tids_for_edge(t);
    std::map<simplex::Edge, std::tuple<bool, char, int, CornerLabels>> parent_state;
    for (const size_t tid : parents) {
        parent_state[simplex_from_tet(tid).opposite_edge(e)] = {
            cells.count(tid) > 0,
            filled[tid],
            cell_group ? (*cell_group)[tid] : -1,
            corner_labels && tid < corner_labels->size() ? (*corner_labels)[tid] : CornerLabels{}};
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
    if (corner_labels) {
        corner_labels->resize(tet_capacity());
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
        const auto& [in_set, parent_filled, parent_group, parent_labels] = it->second;
        filled[tid] = parent_filled;
        if (cell_group) {
            (*cell_group)[tid] = parent_group;
        }
        if (corner_labels) {
            (*corner_labels)[tid] =
                tight_seal::kept_corner_labels(parent_labels, oriented_tet_vids(tid));
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
    std::vector<int>* cell_group,
    std::vector<CornerLabels>* corner_labels)
{
    std::array<size_t, 3> sorted = f;
    std::sort(sorted.begin(), sorted.end());
    const Tuple t = std::get<0>(tuple_from_face(sorted));

    // The tets on the face give three children each, which keep the tet's corner off the face.
    // Record by that corner whether the tet is in the set, whether it was filled, its group and
    // its corner labels.
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
    std::map<size_t, std::tuple<bool, char, int, CornerLabels>> parent_state;
    for (const size_t tid : parents) {
        parent_state[off_face(tid)] = {
            cells.count(tid) > 0,
            filled[tid],
            cell_group ? (*cell_group)[tid] : -1,
            corner_labels && tid < corner_labels->size() ? (*corner_labels)[tid] : CornerLabels{}};
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
    if (corner_labels) {
        corner_labels->resize(tet_capacity());
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
        const auto& [in_set, parent_filled, parent_group, parent_labels] = it->second;
        filled[tid] = parent_filled;
        if (cell_group) {
            (*cell_group)[tid] = parent_group;
        }
        if (corner_labels) {
            (*corner_labels)[tid] =
                tight_seal::kept_corner_labels(parent_labels, oriented_tet_vids(tid));
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

size_t SimWildMesh::tight_seal_split_tet_tracked(
    const size_t tid,
    std::unordered_set<size_t>& cells,
    std::vector<char>& filled,
    std::vector<int>* cell_group,
    std::vector<CornerLabels>* corner_labels)
{
    const bool in_set = cells.count(tid) > 0;
    const char parent_filled = filled[tid];
    const int parent_group = cell_group ? (*cell_group)[tid] : -1;
    const CornerLabels parent_labels =
        corner_labels && tid < corner_labels->size() ? (*corner_labels)[tid] : CornerLabels{};
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
    if (corner_labels) {
        corner_labels->resize(tet_capacity());
    }
    for (const size_t child : get_one_ring_tids_for_vertex(v_new)) {
        filled[child] = parent_filled;
        if (cell_group) {
            (*cell_group)[child] = parent_group;
        }
        if (corner_labels) {
            (*corner_labels)[child] =
                tight_seal::kept_corner_labels(parent_labels, oriented_tet_vids(child));
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
    Vector3d lo = Vector3d::Constant(std::numeric_limits<double>::infinity());
    Vector3d hi = -lo;
    for (const Tuple& t : get_vertices()) {
        lo = lo.cwiseMin(pos(t.vid(*this)));
        hi = hi.cwiseMax(pos(t.vid(*this)));
    }
    const double tie = 1e-10 * (hi - lo).norm(); // distances this close count as equal

    // A vertex's nearest region of a component (the first on an exact tie) and its tie set: the
    // regions within `tie` of the nearest, ascending.
    struct Nearest
    {
        size_t region;
        std::vector<size_t> ties;
    };
    std::vector<std::map<size_t, Nearest>> nearest(components.size());
    const auto nearest_of = [&](const size_t c, const size_t v) -> const Nearest& {
        auto it = nearest[c].find(v);
        if (it == nearest[c].end()) {
            const std::vector<size_t>& regions = components_regions[c];
            std::vector<double> d(regions.size());
            for (size_t k = 0; k < regions.size(); ++k) {
                d[k] = tight_seal::bvh_distance(region_bvhs[regions[k]], pos(v));
            }
            const size_t best = std::min_element(d.begin(), d.end()) - d.begin();
            Nearest n{regions[best], {}};
            for (size_t k = 0; k < regions.size(); ++k) {
                if (d[k] - d[best] < tie) {
                    n.ties.push_back(regions[k]);
                }
            }
            std::sort(n.ties.begin(), n.ties.end());
            it = nearest[c].emplace(v, std::move(n)).first;
        }
        return it->second;
    };

    // Each tet's own labels of its corners: its corner's nearest region, but the corners of a face
    // it shares with a tet of one of the component's regions (a tet with that region's tag alone)
    // take that region. The queues come from these, all before any split: the tets with three or
    // four labels, the faces with three in a tet holding them, the edges whose ends differ in a
    // tet holding them, with the label pairs asked.
    std::vector<CornerLabels> corner_labels(tet_capacity());
    struct Queues
    {
        std::set<std::array<size_t, 4>> tets;
        std::set<std::array<size_t, 3>> faces;
        std::map<std::array<size_t, 2>, std::set<std::pair<size_t, size_t>>> edges;
    };
    std::vector<Queues> queues(components.size());
    size_t n_on_region = 0, n_on_two = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        std::map<int64_t, size_t> region_of_tag;
        for (const size_t i : components_regions[c]) {
            region_of_tag[tags[i]] = i;
        }
        Queues& q = queues[c];
        for (const size_t tid : components[c].cells) {
            std::array<size_t, 4> vs = oriented_tet_vids(tid);
            std::sort(vs.begin(), vs.end());
            std::map<size_t, size_t> label;
            for (const size_t v : vs) {
                label[v] = nearest_of(c, v).region;
            }
            int on = 0;
            for (int j = 0; j < 4; ++j) {
                const Tuple f = tuple_from_face(tid, j);
                const auto opp = f.switch_tetrahedron(*this);
                if (!opp) {
                    continue;
                }
                const CellTag& beyond = m_tet_attribute[opp->tid(*this)].tags;
                if (beyond.size() != 1) {
                    continue;
                }
                const auto r = region_of_tag.find(*beyond.begin());
                if (r == region_of_tag.end()) {
                    continue;
                }
                if (on++ == 0) {
                    for (const size_t v : get_face_vids(f)) {
                        label[v] = r->second;
                    }
                }
            }
            n_on_region += on > 0;
            n_on_two += on > 1;
            CornerLabels& own = corner_labels[tid];
            own.assign(label.begin(), label.end());
            std::set<size_t> distinct;
            for (const auto& [v, l] : label) {
                distinct.insert(l);
            }
            if (distinct.size() >= 3) {
                q.tets.insert(vs);
            }
            for (int skip = 0; skip < 4; ++skip) {
                std::array<size_t, 3> f;
                for (int j = 0, k = 0; j < 4; ++j) {
                    if (j != skip) {
                        f[k++] = vs[j];
                    }
                }
                if (label[f[0]] != label[f[1]] && label[f[0]] != label[f[2]] &&
                    label[f[1]] != label[f[2]]) {
                    q.faces.insert(f);
                }
            }
            for (const auto& [a, b] : tight_seal::tet_edges) {
                if (label[vs[a]] != label[vs[b]]) {
                    q.edges[{{vs[a], vs[b]}}].insert({label[vs[a]], label[vs[b]]});
                }
            }
        }
    }
    if (n_on_two > 0) {
        logger().warn(
            "tight_seal_geom: split_fill: {} tets share faces with two regions' tets or more (the "
            "regions are not simplicially embedded there); the first face's region labels its "
            "corners",
            n_on_two);
    }

    // The splits, component by component, each queue in turn: the tets and faces at their
    // centroid, the edges where the distances to a pair of labels asked for are equal (the
    // bisection of split_adjust_position on m_voronoi_split_fn). An edge is not split if that
    // falls on an end for every pair asked; split_fill_label settles it. Components meet at edges
    // and vertices, so each tet's component is tracked through the splits in cell_comp.
    std::vector<int> cell_comp(tet_capacity(), -1);
    std::unordered_set<size_t> cells;
    for (size_t c = 0; c < components.size(); ++c) {
        for (const size_t tid : components[c].cells) {
            cell_comp[tid] = int(c);
            cells.insert(tid);
        }
    }
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
                tight_seal_split_tet_tracked(*tid, cells, filled, &cell_comp, &corner_labels) ==
                size_t(-1)) {
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
            if (tight_seal_split_face_tracked(
                    f,
                    centroid,
                    cells,
                    filled,
                    &cell_comp,
                    &corner_labels) == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_faces;
            }
        }
        for (const auto& [ends, pairs] : q.edges) {
            const size_t a = ends[0];
            const size_t b = ends[1];
            if (get_incident_tids_for_edge(a, b).empty()) {
                ++n_gone;
                continue;
            }
            std::function<double(const Vector3d&)> difference;
            for (const auto& pair : pairs) {
                const size_t i = pair.first;
                const size_t j = pair.second;
                std::function<double(const Vector3d&)> d = [&, i, j](const Vector3d& p) {
                    return tight_seal::bvh_distance(region_bvhs[i], p) -
                           tight_seal::bvh_distance(region_bvhs[j], p);
                };
                if (d(pos(a)) <= -tie && d(pos(b)) >= tie) {
                    difference = d;
                    break;
                }
            }
            if (!difference) {
                ++n_tied;
                continue;
            }
            m_voronoi_split_fn = difference;
            const size_t v = tight_seal_split_tracked(
                simplex::Edge(a, b),
                cells,
                filled,
                [](const size_t, const size_t) {},
                &cell_comp,
                &corner_labels);
            m_voronoi_split_fn = nullptr;
            if (v == size_t(-1)) {
                ++n_refused;
            } else {
                ++n_edges;
            }
        }
    }
    logger().info(
        "tight_seal_geom: split_fill split {} tets, {} faces and {} edges of {} components ({} "
        "tets with a face on a region); {} queued elements were gone (an edge shared with a "
        "component split before), {} edges were not split because the equality fell on an end, "
        "{} splits were refused",
        n_tets,
        n_faces,
        n_edges,
        components.size(),
        n_on_region,
        n_gone,
        n_tied,
        n_refused);

    // Each tet of the components takes a region from its corners' own labels.
    size_t n_agree = 0, n_ties = 0, n_arbitrary = 0;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (tid >= cell_comp.size() || cell_comp[tid] < 0) {
            continue;
        }
        const size_t c = size_t(cell_comp[tid]);
        const std::pair<size_t, int> chosen = tight_seal::split_fill_label(
            corner_labels[tid],
            [&](const size_t v) -> const std::vector<size_t>& { return nearest_of(c, v).ties; });
        CellTag& cell_tags = m_tet_attribute[tid].tags;
        for (const size_t j : components_regions[c]) {
            cell_tags.erase(tags[j]);
        }
        cell_tags.insert(tags[chosen.first]);
        filled[tid] = 1;
        ++(chosen.second == 0 ? n_agree : chosen.second == 1 ? n_ties : n_arbitrary);
    }
    logger().info(
        "tight_seal_geom: split_fill labelled {} tets: {} by their corners' own labels, {} by the "
        "one region their corners' tie sets share, {} arbitrarily (an exact tie between regions)",
        n_agree + n_ties + n_arbitrary,
        n_agree,
        n_ties,
        n_arbitrary);
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
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
