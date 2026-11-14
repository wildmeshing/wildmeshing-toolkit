#include "TightSealGeom.hpp"

#include "MaterialInterface.hpp"
#include "SimWildMesh.h"
#include "SimWildMeshTri.hpp"

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldCheck.hpp>

#include <Eigen/LU>

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

std::optional<Eigen::VectorXd> equal_distances_point(const Eigen::MatrixXd& D)
{
    // sum_c lambda_c (D(0, c) - D(r, c)) = 0 for r = 1..k-1, and sum_c lambda_c = 1
    const Eigen::Index k = D.cols();
    Eigen::MatrixXd A(k, k);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(k);
    for (Eigen::Index r = 1; r < k; ++r) {
        A.row(r - 1) = D.row(0) - D.row(r);
    }
    A.row(k - 1).setOnes();
    b(k - 1) = 1;
    const Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    if (!lu.isInvertible()) {
        return std::nullopt;
    }
    Eigen::VectorXd lambda = lu.solve(b);
    if (!lambda.allFinite()) {
        return std::nullopt;
    }
    return lambda;
}

void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles)
{
    for (size_t k = 1; k + 1 < poly.size(); ++k) {
        triangles.push_back(poly[0]);
        triangles.push_back(poly[k]);
        triangles.push_back(poly[k + 1]);
    }
}

namespace {

/// tight_seal_topology_check on either mesh. `cells`: the mesh's cells (ids); `tags_now(id)`: a
/// cell's tags; `neighbours(id)`: the cells sharing a facet with it; `nonmanifold(cells, in)`:
/// the number of vertices of `cells` not manifold in the cells for which `in` holds.
void topology_check(
    const std::vector<size_t>& cells,
    const std::function<const CellTag&(size_t)>& tags_now,
    const std::vector<char>& filled,
    const std::vector<CellTag>& tags_before,
    const std::function<std::vector<size_t>(size_t)>& neighbours,
    const std::function<size_t(const std::vector<size_t>&, const wmtk::utils::CellTest&)>&
        nonmanifold,
    const std::map<int64_t, std::string>& tag_id_to_name,
    const std::string& cell_name)
{
    const auto is_filled = [&](const size_t id) { return id < filled.size() && filled[id]; };
    // A cell's tags before the fill: the fill changes the tags of the cells it labels and of no
    // others, and each split's children take their parent's tags.
    const auto before = [&](const size_t id) -> const CellTag& {
        return is_filled(id) ? tags_before[id] : tags_now(id);
    };

    // the filled cells whose membership of each tag changed: the fill added the tag or took it
    std::map<int64_t, std::vector<size_t>> changed_cells;
    for (const size_t id : cells) {
        if (!is_filled(id)) {
            continue;
        }
        const CellTag& a = tags_before[id];
        const CellTag& b = tags_now(id);
        std::vector<int64_t> diff;
        std::set_symmetric_difference(
            a.begin(),
            a.end(),
            b.begin(),
            b.end(),
            std::back_inserter(diff));
        for (const int64_t tag : diff) {
            changed_cells[tag].push_back(id);
        }
    }
    if (changed_cells.empty()) {
        return;
    }

    // the cells of each changed tag, before the fill or after it
    std::map<int64_t, std::vector<size_t>> cells_of;
    size_t capacity = 0;
    for (const size_t id : cells) {
        capacity = std::max(capacity, id + 1);
        for (const CellTag* t : {&before(id), &tags_now(id)}) {
            for (const int64_t tag : *t) {
                if (changed_cells.count(tag) == 0) {
                    continue;
                }
                std::vector<size_t>& of = cells_of[tag];
                if (of.empty() || of.back() != id) {
                    of.push_back(id);
                }
            }
        }
    }

    // Labels the components, through facets, of the cells of `of` for which `in` holds; returns
    // their number.
    const auto label = [&](const std::vector<size_t>& of,
                           const std::function<bool(size_t)>& in,
                           std::vector<int>& comp) {
        int n = 0;
        for (const size_t start : of) {
            if (comp[start] >= 0 || !in(start)) {
                continue;
            }
            comp[start] = n;
            std::vector<size_t> queue{start};
            for (size_t i = 0; i < queue.size(); ++i) { // BFS loop with queue growing
                for (const size_t nbr : neighbours(queue[i])) {
                    if (comp[nbr] < 0 && in(nbr)) {
                        comp[nbr] = n;
                        queue.push_back(nbr);
                    }
                }
            }
            ++n;
        }
        return size_t(n);
    };

    std::vector<int> comp_before(capacity, -1);
    std::vector<int> comp_after(capacity, -1);
    size_t n_warned = 0;
    for (const auto& [tag, changed] : changed_cells) {
        const std::vector<size_t>& of = cells_of[tag];
        const std::function<bool(size_t)> in_before = [&](const size_t id) {
            return before(id).count(tag) > 0;
        };
        const std::function<bool(size_t)> in_after = [&](const size_t id) {
            return tags_now(id).count(tag) > 0;
        };
        // A vertex or an edge changes only if a cell holding it does, so the vertices of the
        // changed cells are enough.
        const int64_t nm_change =
            int64_t(nonmanifold(changed, in_after)) - int64_t(nonmanifold(changed, in_before));

        // Each component after the fill holds the components before it whose cells (carrying
        // the tag both before and after) it holds, and the reverse.
        const size_t n_before = label(of, in_before, comp_before);
        const size_t n_after = label(of, in_after, comp_after);
        std::vector<std::set<int>> held(n_after);
        std::vector<std::set<int>> holders(n_before);
        for (const size_t id : of) {
            if (comp_before[id] >= 0 && comp_after[id] >= 0) {
                held[comp_after[id]].insert(comp_before[id]);
                holders[comp_before[id]].insert(comp_after[id]);
            }
            comp_before[id] = -1;
            comp_after[id] = -1;
        }
        size_t islands = 0, merges = 0, splits = 0, lost = 0;
        for (const std::set<int>& h : held) {
            if (h.empty()) {
                ++islands;
            } else {
                merges += h.size() - 1;
            }
        }
        for (const std::set<int>& h : holders) {
            if (h.empty()) {
                ++lost;
            } else {
                splits += h.size() - 1;
            }
        }

        const auto it = tag_id_to_name.find(tag);
        const std::string name = it == tag_id_to_name.end() ? std::to_string(tag) : it->second;
        if (nm_change != 0 || islands > 0 || merges > 0) {
            ++n_warned;
            logger().warn(
                "tight_seal_geom topology check: tag {}: (non-manifold vertices, islands, merges) "
                "= ({:+}, {}, {})",
                name,
                nm_change,
                islands,
                merges);
        }
        if (splits > 0 || lost > 0) {
            logger().warn(
                "tight_seal_geom topology check: tag {}: the fill took it from {} that carried it "
                "with another tag, which split its components into {} more pieces and removed {} "
                "of them",
                name,
                cell_name,
                splits,
                lost);
        }
    }
    logger().info(
        "tight_seal_geom topology check: the fill changed {} tags; {} of them have (non-manifold "
        "vertices, islands, merges) other than (0, 0, 0)",
        changed_cells.size(),
        n_warned);
}

} // namespace

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
    // the point of edge (v_out, v_in) where d(x, R) is d/2, v_in being in U (sphere traced from
    // v_out, then bisected; tight_seal::level_crossing); v_in itself if no crossing is found
    std::map<simplex::Edge, Vector2d> traced;
    const auto crossing = [&](const size_t v_out, const size_t v_in) -> Vector2d {
        const simplex::Edge e(v_out, v_in);
        const auto it = traced.find(e);
        if (it != traced.end()) {
            return it->second;
        }
        const std::optional<Vector2d> x =
            tight_seal::level_crossing(dist_R, pos(v_out), pos(v_in), radius, tol, true);
        if (!x) {
            ++n_failed;
        }
        return traced[e] = x ? *x : pos(v_in);
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
    const std::vector<CellTag>& fill_tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    const std::function<int(size_t)>& color_of,
    const std::vector<char>& in_J,
    const int64_t max_rounds,
    std::vector<char>& filled,
    std::vector<CellTag>& tags_before)
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
    const auto dist = [&](const size_t region, const Vector2d& p) {
        return tight_seal::bvh_distance(region_bvhs[region], p);
    };

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
                d[k] = dist(regions[k], pos(v));
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
    // Whether a region of component c other than those of `own` is nearer to p than all of them,
    // by more than `tie`: then p's nearest region is none of `own`.
    const auto other_nearer = [&](const size_t c, const Vector2d& p, const auto& own) {
        double d_own = std::numeric_limits<double>::infinity();
        double d_other = d_own;
        for (const size_t region : components_regions[c]) {
            const bool is_own = std::find(own.begin(), own.end(), region) != own.end();
            double& best = is_own ? d_own : d_other;
            best = std::min(best, dist(region, p));
        }
        return d_other < d_own - tie;
    };
    // A face's triple point: where the distances to the regions `labels` (one per corner, in the
    // order of the corners), interpolated linearly from the corners, are equal. Nothing unless it
    // lies inside the face.
    const auto triple_point = [&](const std::array<size_t, 3>& f,
                                  const std::array<size_t, 3>& labels) -> std::optional<Vector2d> {
        Eigen::MatrixXd D(3, 3);
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                D(r, k) = dist(labels[r], pos(f[k]));
            }
        }
        const std::optional<Eigen::VectorXd> lambda = tight_seal::equal_distances_point(D);
        if (!lambda || !(lambda->array() > 0).all()) {
            return std::nullopt;
        }
        return Vector2d(
            (*lambda)(0) * pos(f[0]) + (*lambda)(1) * pos(f[1]) + (*lambda)(2) * pos(f[2]));
    };
    const auto find_face = [&](const std::array<size_t, 3>& vs) -> std::optional<size_t> {
        for (const size_t f : get_one_ring_fids_for_vertex(vs[0])) {
            std::array<size_t, 3> ws = oriented_tri_vids(f);
            std::sort(ws.begin(), ws.end());
            if (ws == vs) {
                return f;
            }
        }
        return std::nullopt;
    };

    // Each component's faces, tracked through every split below: components meet at vertices
    // only, so each has a set of its own.
    std::vector<std::unordered_set<size_t>> comp_faces(components.size());
    for (size_t c = 0; c < components.size(); ++c) {
        comp_faces[c].insert(components[c].cells.begin(), components[c].cells.end());
    }

    // Preprocessing (tight seal v2 note, 4a), so that a face with three labels holds their triple
    // point; as in SimWildMesh::tight_seal_split_fill. Labels here are the vertices' nearest
    // regions. A round marks each edge whose ends' labels differ if, where the distances to the
    // two are equal, another region of the component is nearest, and each face with three labels
    // and no marked edge if, at its triple point inside it, another region is nearest. Then it
    // splits the marked faces at their points and the marked edges at theirs, so each new vertex
    // shows the region found nearest there. After the first round only the edges and faces
    // touching a new vertex can change.
    if (max_rounds > 0) {
        size_t n_pre_faces = 0, n_pre_edges = 0, n_pre_refused = 0, n_pre_gone = 0;
        int64_t round = 0;
        std::set<size_t> fresh; // the vertices the last round made
        while (true) {
            std::set<std::pair<size_t, std::array<size_t, 2>>> edges_to_check;
            std::set<std::pair<size_t, std::array<size_t, 3>>> faces_to_check;
            const auto is_new = [&](const size_t v) { return round == 0 || fresh.count(v) > 0; };
            const auto collect = [&](const size_t c, const size_t fid) {
                std::array<size_t, 3> vs = oriented_tri_vids(fid);
                std::sort(vs.begin(), vs.end());
                std::array<size_t, 3> label;
                for (int j = 0; j < 3; ++j) {
                    label[j] = nearest_of(c, vs[j]).region;
                }
                for (int j = 0; j < 3; ++j) {
                    for (int k = j + 1; k < 3; ++k) {
                        if ((is_new(vs[j]) || is_new(vs[k])) && label[j] != label[k]) {
                            edges_to_check.insert({c, {{vs[j], vs[k]}}});
                        }
                    }
                }
                if ((is_new(vs[0]) || is_new(vs[1]) || is_new(vs[2])) && label[0] != label[1] &&
                    label[0] != label[2] && label[1] != label[2]) {
                    faces_to_check.insert({c, vs});
                }
            };
            for (size_t c = 0; c < components.size(); ++c) {
                if (round == 0) {
                    for (const size_t fid : comp_faces[c]) {
                        collect(c, fid);
                    }
                } else {
                    for (const size_t v : fresh) {
                        for (const size_t fid : get_one_ring_fids_for_vertex(v)) {
                            if (comp_faces[c].count(fid) > 0) {
                                collect(c, fid);
                            }
                        }
                    }
                }
            }

            std::map<std::array<size_t, 2>, std::pair<size_t, Vector2d>> marked_edges;
            for (const auto& [c, e] : edges_to_check) {
                if (marked_edges.count(e) > 0) {
                    continue;
                }
                const size_t i = nearest_of(c, e[0]).region;
                const size_t j = nearest_of(c, e[1]).region;
                const std::function<double(const Vector2d&)> difference =
                    [&, i, j](const Vector2d& p) { return dist(i, p) - dist(j, p); };
                if (!(difference(pos(e[0])) <= -tie && difference(pos(e[1])) >= tie)) {
                    continue; // the distances are equal at an end
                }
                const Vector2d x = tight_seal::bisect_sign_change(difference, pos(e[0]), pos(e[1]));
                if (other_nearer(c, x, std::array<size_t, 2>{{i, j}})) {
                    marked_edges.emplace(e, std::make_pair(c, x));
                }
            }
            std::map<std::array<size_t, 3>, std::pair<size_t, Vector2d>> marked_faces;
            for (const auto& [c, f] : faces_to_check) {
                if (marked_faces.count(f) > 0 || marked_edges.count({{f[0], f[1]}}) > 0 ||
                    marked_edges.count({{f[0], f[2]}}) > 0 ||
                    marked_edges.count({{f[1], f[2]}}) > 0) {
                    continue;
                }
                const std::array<size_t, 3> labels = {
                    {nearest_of(c, f[0]).region,
                     nearest_of(c, f[1]).region,
                     nearest_of(c, f[2]).region}};
                const std::optional<Vector2d> p = triple_point(f, labels);
                if (p && other_nearer(c, *p, labels)) {
                    marked_faces.emplace(f, std::make_pair(c, *p));
                }
            }
            if (marked_edges.empty() && marked_faces.empty()) {
                break;
            }
            if (round == max_rounds) {
                logger().warn(
                    "tight_seal_geom: split_fill preprocessing stopped after {} rounds, the "
                    "maximum, with {} edges and {} faces still marked",
                    max_rounds,
                    marked_edges.size(),
                    marked_faces.size());
                break;
            }
            ++round;
            fresh.clear();
            for (const auto& [f, cp] : marked_faces) {
                const std::optional<size_t> fid = find_face(f);
                if (!fid) {
                    ++n_pre_gone;
                    continue;
                }
                const size_t v =
                    tight_seal_split_face_tracked(*fid, cp.second, comp_faces[cp.first], filled);
                if (v == size_t(-1)) {
                    ++n_pre_refused;
                } else {
                    ++n_pre_faces;
                    fresh.insert(v);
                }
            }
            for (const auto& [e, cx] : marked_edges) {
                if (!try_tuple_from_edge({{e[0], e[1]}})) {
                    ++n_pre_gone;
                    continue;
                }
                // linear along the edge and zero at x, so split_adjust_position puts the vertex
                // there
                const Vector2d x = cx.second;
                const Vector2d dir = pos(e[1]) - pos(e[0]);
                m_voronoi_split_fn = [x, dir](const Vector2d& p) { return (p - x).dot(dir); };
                const size_t v = tight_seal_split_tracked(
                    simplex::Edge(e[0], e[1]),
                    comp_faces[cx.first],
                    filled);
                m_voronoi_split_fn = nullptr;
                if (v == size_t(-1)) {
                    ++n_pre_refused;
                } else {
                    ++n_pre_edges;
                    fresh.insert(v);
                }
            }
        }
        logger().info(
            "tight_seal_geom: split_fill preprocessing: {} rounds split {} faces at their triple "
            "point and {} edges where the distances to their ends' colors are equal, another "
            "color being nearest there; {} splits were refused, {} marked elements were gone",
            round,
            n_pre_faces,
            n_pre_edges,
            n_pre_refused,
            n_pre_gone);
    }

    // Each face's own labels of its corners: its corner's nearest region, but the ends of an edge
    // it shares with a face of one of the component's regions (a face with that region's tag
    // alone) take that region. The queues come from these, all before any split: the faces with
    // three labels (with their corners' labels), the edges whose ends differ in a face holding
    // them, with the label pairs asked.
    std::vector<CornerLabels> corner_labels(tri_capacity());
    struct Queues
    {
        std::map<std::array<size_t, 3>, std::array<size_t, 3>> faces;
        std::map<std::array<size_t, 2>, std::set<std::pair<size_t, size_t>>> edges;
    };
    std::vector<Queues> queues(components.size());
    size_t n_on_region = 0, n_on_two = 0;
    // Whether a face is in J: in a component to fill, or in one of J's other components, whose
    // faces no split here cuts (they share no edge with a component to fill). Only a face outside
    // J can be a color.
    const auto in_join = [&](const size_t fid) {
        if (fid < in_J.size() && in_J[fid]) {
            return true;
        }
        for (const std::unordered_set<size_t>& faces : comp_faces) {
            if (faces.count(fid) > 0) {
                return true;
            }
        }
        return false;
    };
    for (size_t c = 0; c < components.size(); ++c) {
        const std::set<size_t> colors(components_regions[c].begin(), components_regions[c].end());
        Queues& q = queues[c];
        std::vector<size_t> fids(comp_faces[c].begin(), comp_faces[c].end());
        std::sort(fids.begin(), fids.end());
        for (const size_t fid : fids) {
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
                const size_t beyond = opp->fid(*this);
                if (in_join(beyond)) {
                    continue; // a face of J is no color
                }
                const int k = color_of(beyond);
                if (k < 0 || colors.count(size_t(k)) == 0) {
                    continue;
                }
                if (on++ == 0) {
                    label[e.vid(*this)] = size_t(k);
                    label[e.switch_vertex(*this).vid(*this)] = size_t(k);
                }
            }
            n_on_region += on > 0;
            n_on_two += on > 1;
            CornerLabels& own = corner_labels[fid];
            own.assign(label.begin(), label.end());
            if (label[vs[0]] != label[vs[1]] && label[vs[0]] != label[vs[2]] &&
                label[vs[1]] != label[vs[2]]) {
                q.faces.emplace(
                    vs,
                    std::array<size_t, 3>{{label[vs[0]], label[vs[1]], label[vs[2]]}});
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
            "tight_seal_geom: split_fill: {} faces share edges with faces of two colors or more; "
            "the first edge's color labels its ends",
            n_on_two);
    }

    // The splits, component by component, each queue in turn: the faces at their triple point
    // (the centroid if it lies outside), the edges where the distances to a pair of labels asked
    // for are equal (the bisection of split_adjust_position on m_voronoi_split_fn). An edge is not
    // split if that falls on an end for every pair asked; split_fill_label settles it.
    size_t n_faces = 0, n_edges = 0, n_gone = 0, n_tied = 0, n_refused = 0;
    size_t n_faces_at_point = 0, n_faces_at_centroid = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const Queues& q = queues[c];
        for (const auto& [vs, labels] : q.faces) {
            const std::optional<size_t> fid = find_face(vs);
            if (!fid) {
                ++n_gone;
                continue;
            }
            const Vector2d centroid = (pos(vs[0]) + pos(vs[1]) + pos(vs[2])) / 3.;
            std::optional<Vector2d> triple = triple_point(vs, labels);
            size_t v_new = tight_seal_split_face_tracked(
                *fid,
                triple ? *triple : centroid,
                comp_faces[c],
                filled,
                &corner_labels);
            if (v_new == size_t(-1) && triple) {
                // refused at the point (a child inverted in doubles): the centroid instead
                triple.reset();
                v_new = tight_seal_split_face_tracked(
                    *fid,
                    centroid,
                    comp_faces[c],
                    filled,
                    &corner_labels);
            }
            if (v_new == size_t(-1)) {
                ++n_refused;
                continue;
            }
            ++n_faces;
            ++(triple ? n_faces_at_point : n_faces_at_centroid);
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
                    return dist(i, p) - dist(j, p);
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
        "an edge on a color); {} queued elements were gone, {} edges were not split because the "
        "equality fell on an end, {} splits were refused",
        n_faces,
        n_edges,
        components.size(),
        n_on_region,
        n_gone,
        n_tied,
        n_refused);
    logger().info(
        "tight_seal_geom: split_fill placed {} faces at their triple point and {} at their "
        "centroid (the point lies outside them, or the split there was refused)",
        n_faces_at_point,
        n_faces_at_centroid);

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
            if (fid >= tags_before.size()) {
                tags_before.resize(tri_capacity());
            }
            tags_before[fid] = cell_tags;
            cell_tags.insert(fill_tags[chosen.first].begin(), fill_tags[chosen.first].end());
            filled[fid] = 1;
            ++(chosen.second == 0 ? n_agree : chosen.second == 1 ? n_ties : n_arbitrary);
        }
    }
    logger().info(
        "tight_seal_geom: split_fill labelled {} faces: {} by their corners' own labels, {} by the "
        "one color their corners' tie sets share, {} arbitrarily (an exact tie between colors)",
        n_agree + n_ties + n_arbitrary,
        n_agree,
        n_ties,
        n_arbitrary);
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
}

void SimWildMeshTri::tight_seal_topology_check(
    const std::vector<char>& filled,
    const std::vector<CellTag>& tags_before) const
{
    std::vector<size_t> fids;
    for (const Tuple& t : get_faces()) {
        fids.push_back(t.fid(*this));
    }
    tight_seal::topology_check(
        fids,
        [&](const size_t fid) -> const CellTag& { return m_face_attribute[fid].tags; },
        filled,
        tags_before,
        [&](const size_t fid) {
            std::vector<size_t> nbrs;
            for (int j = 0; j < 3; ++j) {
                if (const auto opp = tuple_from_edge(fid, j).switch_face(*this)) {
                    nbrs.push_back(opp->fid(*this));
                }
            }
            return nbrs;
        },
        [&](const std::vector<size_t>& cells, const wmtk::utils::CellTest& in) {
            return wmtk::utils::nonmanifold_vertices(*this, cells, in).size();
        },
        m_tag_id_to_name,
        "faces");
}

void SimWildMeshTri::tight_seal_geom_sanity(
    const std::vector<CellTag>& fill_tags,
    const std::function<int(size_t)>& color_of,
    const std::vector<char>& filled) const
{
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
        const bool holds_a_set =
            std::any_of(fill_tags.begin(), fill_tags.end(), [&](const CellTag& set) {
                return std::includes(tag.begin(), tag.end(), set.begin(), set.end());
            });
        if (!holds_a_set) {
            log_and_throw_error(
                "tight_seal_geom sanity check: filled face {} has tags {}, holding none of the "
                "fill tag sets",
                fid,
                std::vector<int64_t>(tag.begin(), tag.end()));
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
        std::set<int> borders;
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
                const int k = color_of(nbr);
                if (k >= 0) {
                    borders.insert(k);
                }
            }
        }
        ++n_parts;
        if (borders.size() < 2) {
            log_and_throw_error(
                "tight_seal_geom sanity check: a filled part of {} faces borders only the "
                "colors {}, so a concave feature of one body was filled",
                part.size(),
                std::vector<int>(borders.begin(), borders.end()));
        }
    }
    logger().info(
        "tight_seal_geom sanity check passed: no face inverted; {} filled faces in {} "
        "parts, each bordering two colors or more",
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
    // the point of edge (v_out, v_in) where d(x, R) is d/2, v_in being in U (sphere traced from
    // v_out, then bisected; tight_seal::level_crossing); v_in itself if no crossing is found
    std::map<simplex::Edge, Vector3d> traced;
    const auto crossing = [&](const size_t v_out, const size_t v_in) -> Vector3d {
        const simplex::Edge e(v_out, v_in);
        const auto it = traced.find(e);
        if (it != traced.end()) {
            return it->second;
        }
        const std::optional<Vector3d> x =
            tight_seal::level_crossing(dist_R, pos(v_out), pos(v_in), radius, tol, true);
        if (!x) {
            ++n_failed;
        }
        return traced[e] = x ? *x : pos(v_in);
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
    const Vector3d& p,
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
    m_tet_split_position = p;
    std::vector<Tuple> new_tets;
    const bool ok = split_tet(t, new_tets);
    m_tet_split_position.reset();
    if (!ok) {
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
    const std::vector<CellTag>& fill_tags,
    const std::vector<SimpleBVH::BVH>& region_bvhs,
    const std::function<int(size_t)>& color_of,
    const std::vector<char>& in_J,
    const int64_t max_rounds,
    std::vector<char>& filled,
    std::vector<CellTag>& tags_before)
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
    const auto dist = [&](const size_t region, const Vector3d& p) {
        return tight_seal::bvh_distance(region_bvhs[region], p);
    };

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
                d[k] = dist(regions[k], pos(v));
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
    // Whether a region of component c other than those of `own` is nearer to p than all of them,
    // by more than `tie`: then p's nearest region is none of `own`.
    const auto other_nearer = [&](const size_t c, const Vector3d& p, const auto& own) {
        double d_own = std::numeric_limits<double>::infinity();
        double d_other = d_own;
        for (const size_t region : components_regions[c]) {
            const bool is_own = std::find(own.begin(), own.end(), region) != own.end();
            double& best = is_own ? d_own : d_other;
            best = std::min(best, dist(region, p));
        }
        return d_other < d_own - tie;
    };
    // Where the distances to the regions `labels` (one per corner, in the order of the corners),
    // interpolated linearly from the corners, are equal: the triple point of a face, the
    // quadruple point of a tet. Nothing unless it lies inside.
    const auto equal_distances_inside = [&](const auto& corners,
                                            const auto& labels) -> std::optional<Vector3d> {
        const Eigen::Index k = Eigen::Index(corners.size());
        Eigen::MatrixXd D(k, k);
        for (Eigen::Index r = 0; r < k; ++r) {
            for (Eigen::Index c = 0; c < k; ++c) {
                D(r, c) = dist(labels[size_t(r)], pos(corners[size_t(c)]));
            }
        }
        const std::optional<Eigen::VectorXd> lambda = tight_seal::equal_distances_point(D);
        if (!lambda || !(lambda->array() > 0).all()) {
            return std::nullopt;
        }
        Vector3d p = Vector3d::Zero();
        for (Eigen::Index c = 0; c < k; ++c) {
            p += (*lambda)(c)*pos(corners[size_t(c)]);
        }
        return p;
    };

    // Each tet's component, tracked through every split below: components meet at edges and
    // vertices, so one split can cut tets of two. -2 for the tets of J's other components (not
    // filled), -1 for the tets outside J: only those can be a color.
    std::vector<int> cell_comp(tet_capacity(), -1);
    std::unordered_set<size_t> cells;
    for (size_t c = 0; c < components.size(); ++c) {
        for (const size_t tid : components[c].cells) {
            cell_comp[tid] = int(c);
            cells.insert(tid);
        }
    }
    for (size_t tid = 0; tid < in_J.size() && tid < cell_comp.size(); ++tid) {
        if (in_J[tid] && cell_comp[tid] == -1) {
            cell_comp[tid] = -2;
        }
    }

    // Preprocessing (tight seal v2 note, 4a), so that a tet with three labels is crossed by their
    // triple line and one with four holds their quadruple point. Labels here are the vertices'
    // nearest regions. A round marks each edge whose ends' labels differ if, where the distances
    // to the two are equal, another region of the component is nearest, and each face with three
    // labels and no marked edge if, at its triple point inside it, another region is nearest.
    // Then it splits the marked faces at their points and the marked edges at theirs, so each new
    // vertex shows the region found nearest there. A face split splits no edge, and a marked face
    // keeps its edges, so every point stays valid through the round. A vertex never changes its
    // label, so after the first round only the edges and faces touching a new vertex can change.
    if (max_rounds > 0) {
        size_t n_pre_faces = 0, n_pre_edges = 0, n_pre_refused = 0, n_pre_gone = 0;
        int64_t round = 0;
        std::set<size_t> fresh; // the vertices the last round made
        while (true) {
            std::set<std::pair<size_t, std::array<size_t, 2>>> edges_to_check;
            std::set<std::pair<size_t, std::array<size_t, 3>>> faces_to_check;
            const auto is_new = [&](const size_t v) { return round == 0 || fresh.count(v) > 0; };
            const auto collect = [&](const size_t tid) {
                const size_t c = size_t(cell_comp[tid]);
                std::array<size_t, 4> vs = oriented_tet_vids(tid);
                std::sort(vs.begin(), vs.end());
                std::array<size_t, 4> label;
                for (int j = 0; j < 4; ++j) {
                    label[j] = nearest_of(c, vs[j]).region;
                }
                for (const auto& [a, b] : tight_seal::tet_edges) {
                    if ((is_new(vs[a]) || is_new(vs[b])) && label[a] != label[b]) {
                        edges_to_check.insert({c, {{vs[a], vs[b]}}});
                    }
                }
                for (int skip = 0; skip < 4; ++skip) {
                    std::array<size_t, 3> f, l;
                    for (int j = 0, k = 0; j < 4; ++j) {
                        if (j != skip) {
                            f[k] = vs[j];
                            l[k++] = label[j];
                        }
                    }
                    if ((is_new(f[0]) || is_new(f[1]) || is_new(f[2])) && l[0] != l[1] &&
                        l[0] != l[2] && l[1] != l[2]) {
                        faces_to_check.insert({c, f});
                    }
                }
            };
            if (round == 0) {
                for (const size_t tid : cells) {
                    collect(tid);
                }
            } else {
                for (const size_t v : fresh) {
                    for (const size_t tid : get_one_ring_tids_for_vertex(v)) {
                        if (cells.count(tid) > 0) {
                            collect(tid);
                        }
                    }
                }
            }

            std::map<std::array<size_t, 2>, Vector3d> marked_edges;
            for (const auto& [c, e] : edges_to_check) {
                if (marked_edges.count(e) > 0) {
                    continue;
                }
                const size_t i = nearest_of(c, e[0]).region;
                const size_t j = nearest_of(c, e[1]).region;
                const std::function<double(const Vector3d&)> difference =
                    [&, i, j](const Vector3d& p) { return dist(i, p) - dist(j, p); };
                if (!(difference(pos(e[0])) <= -tie && difference(pos(e[1])) >= tie)) {
                    continue; // the distances are equal at an end
                }
                const Vector3d x = tight_seal::bisect_sign_change(difference, pos(e[0]), pos(e[1]));
                if (other_nearer(c, x, std::array<size_t, 2>{{i, j}})) {
                    marked_edges.emplace(e, x);
                }
            }
            std::map<std::array<size_t, 3>, Vector3d> marked_faces;
            for (const auto& [c, f] : faces_to_check) {
                if (marked_faces.count(f) > 0 || marked_edges.count({{f[0], f[1]}}) > 0 ||
                    marked_edges.count({{f[0], f[2]}}) > 0 ||
                    marked_edges.count({{f[1], f[2]}}) > 0) {
                    continue;
                }
                const std::array<size_t, 3> labels = {
                    {nearest_of(c, f[0]).region,
                     nearest_of(c, f[1]).region,
                     nearest_of(c, f[2]).region}};
                const std::optional<Vector3d> p = equal_distances_inside(f, labels);
                if (p && other_nearer(c, *p, labels)) {
                    marked_faces.emplace(f, *p);
                }
            }
            if (marked_edges.empty() && marked_faces.empty()) {
                break;
            }
            if (round == max_rounds) {
                logger().warn(
                    "tight_seal_geom: split_fill preprocessing stopped after {} rounds, the "
                    "maximum, with {} edges and {} faces still marked",
                    max_rounds,
                    marked_edges.size(),
                    marked_faces.size());
                break;
            }
            ++round;
            fresh.clear();
            for (const auto& [f, p] : marked_faces) {
                if (!try_tuple_from_face(f)) {
                    ++n_pre_gone;
                    continue;
                }
                const size_t v = tight_seal_split_face_tracked(f, p, cells, filled, &cell_comp);
                if (v == size_t(-1)) {
                    ++n_pre_refused;
                } else {
                    ++n_pre_faces;
                    fresh.insert(v);
                }
            }
            for (const auto& [e, x] : marked_edges) {
                if (get_incident_tids_for_edge(e[0], e[1]).empty()) {
                    ++n_pre_gone;
                    continue;
                }
                // linear along the edge and zero at x, so split_adjust_position puts the vertex
                // there
                const Vector3d dir = pos(e[1]) - pos(e[0]);
                m_voronoi_split_fn = [x, dir](const Vector3d& p) { return (p - x).dot(dir); };
                const size_t v =
                    tight_seal_split_tracked(simplex::Edge(e[0], e[1]), cells, filled, &cell_comp);
                m_voronoi_split_fn = nullptr;
                if (v == size_t(-1)) {
                    ++n_pre_refused;
                } else {
                    ++n_pre_edges;
                    fresh.insert(v);
                }
            }
        }
        logger().info(
            "tight_seal_geom: split_fill preprocessing: {} rounds split {} faces at their triple "
            "point and {} edges where the distances to their ends' colors are equal, another "
            "color being nearest there; {} splits were refused, {} marked elements were gone",
            round,
            n_pre_faces,
            n_pre_edges,
            n_pre_refused,
            n_pre_gone);
    }

    // Each tet's own labels of its corners: its corner's nearest region, but the corners of a face
    // it shares with a tet of one of the component's regions (a tet with that region's tag alone)
    // take that region. The queues come from these, all before any split: the tets with four
    // labels, the faces with three in a tet holding them (both with their corners' labels), the
    // edges whose ends differ in a tet holding them, with the label pairs asked. A tet with three
    // labels is not split itself: its two faces with three are, then its five edges whose ends
    // differ, and the segment between the two faces' points becomes an edge (the second face
    // split happens in the child whose corner off that face is the first face's point).
    std::vector<std::vector<size_t>> comp_cells(components.size());
    for (const size_t tid : cells) {
        comp_cells[size_t(cell_comp[tid])].push_back(tid);
    }
    std::vector<CornerLabels> corner_labels(tet_capacity());
    struct Queues
    {
        std::map<std::array<size_t, 4>, std::array<size_t, 4>> tets;
        std::map<std::array<size_t, 3>, std::array<size_t, 3>> faces;
        std::map<std::array<size_t, 2>, std::set<std::pair<size_t, size_t>>> edges;
    };
    std::vector<Queues> queues(components.size());
    size_t n_on_region = 0, n_on_two = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const std::set<size_t> colors(components_regions[c].begin(), components_regions[c].end());
        Queues& q = queues[c];
        std::sort(comp_cells[c].begin(), comp_cells[c].end());
        for (const size_t tid : comp_cells[c]) {
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
                const size_t beyond = opp->tid(*this);
                if (beyond < cell_comp.size() && cell_comp[beyond] != -1) {
                    continue; // a tet of J is no color
                }
                const int k = color_of(beyond);
                if (k < 0 || colors.count(size_t(k)) == 0) {
                    continue;
                }
                if (on++ == 0) {
                    for (const size_t v : get_face_vids(f)) {
                        label[v] = size_t(k);
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
            if (distinct.size() == 4) {
                q.tets.emplace(
                    vs,
                    std::array<size_t, 4>{
                        {label[vs[0]], label[vs[1]], label[vs[2]], label[vs[3]]}});
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
                    q.faces.emplace(
                        f,
                        std::array<size_t, 3>{{label[f[0]], label[f[1]], label[f[2]]}});
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
            "tight_seal_geom: split_fill: {} tets share faces with tets of two colors or more; the "
            "first face's color labels its corners",
            n_on_two);
    }

    // The splits, component by component, each queue in turn: the tets at their quadruple
    // point, the faces at their triple point (either at its centroid if the point lies outside,
    // or if the split there is refused); the edges where the distances to a pair of labels
    // asked for are equal (the bisection of split_adjust_position on m_voronoi_split_fn). An edge
    // is not split if that falls on an end for every pair asked; split_fill_label settles it.
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
    size_t n_tets_at_point = 0, n_tets_at_centroid = 0;
    size_t n_faces_at_point = 0, n_faces_at_centroid = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const Queues& q = queues[c];
        for (const auto& [vs, labels] : q.tets) {
            const std::optional<size_t> tid = find_tet(vs);
            if (!tid) {
                ++n_gone;
                continue;
            }
            const Vector3d centroid = (pos(vs[0]) + pos(vs[1]) + pos(vs[2]) + pos(vs[3])) / 4.;
            std::optional<Vector3d> quadruple = equal_distances_inside(vs, labels);
            size_t v_new = tight_seal_split_tet_tracked(
                *tid,
                quadruple ? *quadruple : centroid,
                cells,
                filled,
                &cell_comp,
                &corner_labels);
            if (v_new == size_t(-1) && quadruple) {
                // refused at the point (a child inverted in doubles): the centroid instead
                quadruple.reset();
                v_new = tight_seal_split_tet_tracked(
                    *tid,
                    centroid,
                    cells,
                    filled,
                    &cell_comp,
                    &corner_labels);
            }
            if (v_new == size_t(-1)) {
                ++n_refused;
                continue;
            }
            ++n_tets;
            ++(quadruple ? n_tets_at_point : n_tets_at_centroid);
        }
        for (const auto& [f, labels] : q.faces) {
            if (!try_tuple_from_face(f)) {
                ++n_gone;
                continue;
            }
            const Vector3d centroid = (pos(f[0]) + pos(f[1]) + pos(f[2])) / 3.;
            std::optional<Vector3d> triple = equal_distances_inside(f, labels);
            size_t v_new = tight_seal_split_face_tracked(
                f,
                triple ? *triple : centroid,
                cells,
                filled,
                &cell_comp,
                &corner_labels);
            if (v_new == size_t(-1) && triple) {
                // refused at the point (a child inverted in doubles): the centroid instead
                triple.reset();
                v_new = tight_seal_split_face_tracked(
                    f,
                    centroid,
                    cells,
                    filled,
                    &cell_comp,
                    &corner_labels);
            }
            if (v_new == size_t(-1)) {
                ++n_refused;
                continue;
            }
            ++n_faces;
            ++(triple ? n_faces_at_point : n_faces_at_centroid);
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
                    return dist(i, p) - dist(j, p);
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
        "tets with a face on a color); {} queued elements were gone (an edge shared with a "
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
    logger().info(
        "tight_seal_geom: split_fill placed {} tets at their quadruple point and {} at their "
        "centroid, {} faces at their triple point and {} at their centroid (the centroid if the "
        "point lies outside, or the split there was refused)",
        n_tets_at_point,
        n_tets_at_centroid,
        n_faces_at_point,
        n_faces_at_centroid);

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
        if (tid >= tags_before.size()) {
            tags_before.resize(tet_capacity());
        }
        tags_before[tid] = cell_tags;
        cell_tags.insert(fill_tags[chosen.first].begin(), fill_tags[chosen.first].end());
        filled[tid] = 1;
        ++(chosen.second == 0 ? n_agree : chosen.second == 1 ? n_ties : n_arbitrary);
    }
    logger().info(
        "tight_seal_geom: split_fill labelled {} tets: {} by their corners' own labels, {} by the "
        "one color their corners' tie sets share, {} arbitrarily (an exact tie between colors)",
        n_agree + n_ties + n_arbitrary,
        n_agree,
        n_ties,
        n_arbitrary);
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
}

size_t SimWildMesh::tight_seal_new_vertex(
    const Vector3r& position,
    const std::vector<size_t>& carrier)
{
    const size_t v = add_vertex_slot();
    VertexAttributes va(position);
    va.m_is_rounded = false;
    va.m_is_on_surface = false;
    va.m_order = 0;
    va.on_bbox_faces = m_vertex_attribute[carrier.front()].on_bbox_faces;
    va.m_sizing_scalar = 0;
    for (const size_t u : carrier) {
        va.on_bbox_faces =
            wmtk::set_intersection(va.on_bbox_faces, m_vertex_attribute[u].on_bbox_faces);
        va.m_sizing_scalar += m_vertex_attribute[u].m_sizing_scalar / double(carrier.size());
    }
    va.partition_id = m_vertex_attribute[carrier.front()].partition_id;
    m_vertex_attribute[v] = va;
    if (m_track_feature_points) {
        m_feature_point_attribute[v].m_feature_point_id = NO_FEATURE_POINT;
    }
    return v;
}

std::vector<size_t> SimWildMesh::tight_seal_replace_tets(
    const std::vector<size_t>& old_tets,
    const std::vector<std::array<size_t, 4>>& new_tets,
    const std::vector<TetAttributes>& new_attributes,
    const std::function<std::vector<size_t>(const std::vector<size_t>&)>& span)
{
    const auto sorted_face = [](const std::array<size_t, 4>& vs, const int k) {
        std::array<size_t, 3> f{{vs[(k + 1) % 4], vs[(k + 2) % 4], vs[(k + 3) % 4]}};
        std::sort(f.begin(), f.end());
        return f;
    };

    // the old tets' faces with their attributes, read while their ids are still the old ones
    std::map<std::array<size_t, 3>, FaceAttributes> old_faces;
    for (const size_t tid : old_tets) {
        const std::array<size_t, 4> vs = oriented_tet_vids(tid);
        for (int k = 0; k < 4; ++k) {
            const std::array<size_t, 3> f = sorted_face(vs, k);
            if (old_faces.count(f) == 0) {
                old_faces.emplace(f, m_face_attribute[std::get<1>(tuple_from_face(f))]);
            }
        }
    }
    std::map<std::array<size_t, 2>, FeatureEdgeAttributes> old_edges;
    if (m_track_feature_edges) {
        feature_edges_cache(old_tets, old_edges);
    }
    std::vector<size_t> ids;
    if (!replace_tets(old_tets, new_tets, ids)) {
        log_and_throw_error(
            "tight_seal_geom: the new tets do not replace the old ones exactly "
            "(TetMesh::replace_tets refused them; its warning says why)");
    }

    for (size_t i = 0; i < new_tets.size(); ++i) {
        m_tet_attribute[ids[i]] = new_attributes[i];
        m_tet_attribute[ids[i]].m_quality = get_quality(tuple_from_tet(ids[i]));
    }
    // each face at its owner's slot, now that the tets have their new ids
    for (size_t i = 0; i < new_tets.size(); ++i) {
        for (int k = 0; k < 4; ++k) {
            const std::array<size_t, 3> f = sorted_face(new_tets[i], k);
            FaceAttributes& attributes = m_face_attribute[std::get<1>(tuple_from_face(f))];
            auto it = old_faces.find(f);
            if (it == old_faces.end()) {
                const std::vector<size_t> parent = span({f[0], f[1], f[2]});
                if (parent.size() == 3) {
                    it = old_faces.find({{parent[0], parent[1], parent[2]}});
                }
            }
            if (it != old_faces.end()) {
                attributes = it->second;
            } else {
                attributes.reset();
            }
        }
    }
    if (m_track_feature_edges) {
        // the pieces of an old edge take its feature attributes; other new edges none
        std::map<std::array<size_t, 2>, FeatureEdgeAttributes> edges = old_edges;
        for (const std::array<size_t, 4>& t : new_tets) {
            for (int i = 0; i < 4; ++i) {
                for (int j = i + 1; j < 4; ++j) {
                    const std::array<size_t, 2> e{{std::min(t[i], t[j]), std::max(t[i], t[j])}};
                    if (edges.count(e) != 0) {
                        continue;
                    }
                    const std::vector<size_t> parent = span({e[0], e[1]});
                    if (parent.size() == 2) {
                        const auto it = old_edges.find({{parent[0], parent[1]}});
                        if (it != old_edges.end()) {
                            edges.emplace(e, it->second);
                        }
                    }
                }
            }
        }
        feature_edges_restore_region(ids, edges);
    }
    return ids;
}

void SimWildMesh::tight_seal_mi_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_colors,
    const std::vector<CellTag>& fill_tags,
    const std::vector<SimpleBVH::BVH>& color_bvhs,
    std::vector<char>& filled,
    std::vector<CellTag>& tags_before)
{
    namespace mi = material_interface;
    using Face3 = std::array<size_t, 3>;
    const auto sorted3 = [](const size_t a, const size_t b, const size_t c) {
        Face3 f{{a, b, c}};
        std::sort(f.begin(), f.end());
        return f;
    };
    const auto pos = [&](const size_t v) { return m_vertex_attribute[v].pos(); };
    // det[b - a, c - a, d - a] of the tet's corners, exactly: > 0 for a valid tet
    const auto orientation = [&](const std::array<size_t, 4>& t) {
        const Vector3r a = pos(t[0]);
        const Vector3r b = pos(t[1]);
        const Vector3r c = pos(t[2]);
        const Vector3r d = pos(t[3]);
        std::array<std::array<Rational, 3>, 3> m;
        for (int k = 0; k < 3; ++k) {
            m[0][k] = b[k] - a[k];
            m[1][k] = c[k] - a[k];
            m[2][k] = d[k] - a[k];
        }
        const Rational det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                             m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                             m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        return det.get_sign();
    };
    const auto centroid = [&](const std::vector<size_t>& vs) {
        std::array<Rational, 3> x;
        for (const size_t v : vs) {
            const Vector3r p = pos(v);
            for (int k = 0; k < 3; ++k) {
                x[k] = x[k] + p[k];
            }
        }
        const Rational n(double(vs.size()));
        return Vector3r(x[0] / n, x[1] / n, x[2] / n);
    };

    // the component of each tet waiting to be filled (-1: none); a tet that replaces one waiting
    // takes its component, so that a fill reaching into another part leaves it whole
    std::vector<int64_t> comp_of(tet_capacity(), -1);
    for (size_t c = 0; c < components.size(); ++c) {
        for (const size_t tid : components[c].cells) {
            comp_of[tid] = int64_t(c);
        }
    }
    size_t n_cells = 0, n_diagram_vertices = 0, n_centroids = 0, n_beside = 0, n_filled = 0,
           n_rounded = 0, n_new_vertices = 0;
    for (size_t c = 0; c < components.size(); ++c) {
        const std::vector<size_t>& colors = components_colors[c];
        std::vector<size_t> comp_tids;
        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            if (tid < comp_of.size() && comp_of[tid] == int64_t(c)) {
                comp_tids.push_back(tid);
            }
        }
        if (comp_tids.empty()) {
            continue;
        }
        std::vector<std::array<size_t, 4>> tets;
        for (const size_t tid : comp_tids) {
            tets.push_back(oriented_tet_vids(tid));
        }

        // the distances to the component's colors at its vertices: the diagram's functions
        std::map<size_t, std::vector<double>> distances;
        const std::function<double(size_t, size_t)> value = [&](const size_t v, const size_t f) {
            auto it = distances.find(v);
            if (it == distances.end()) {
                std::vector<double> d(colors.size());
                for (size_t k = 0; k < colors.size(); ++k) {
                    d[k] = tight_seal::bvh_distance(
                        color_bvhs[colors[k]],
                        m_vertex_attribute[v].m_posf);
                }
                it = distances.emplace(v, std::move(d)).first;
            }
            return it->second[f];
        };
        const mi::MeshPartition mp = mi::partition_mesh(tets, colors.size(), value);
        n_cells += mp.cells.size();

        // each diagram vertex's mesh vertex: a mesh vertex, or a new one at its exact position
        std::vector<size_t> vertex_of(mp.vertices.size());
        std::map<size_t, std::vector<size_t>> carrier; // new vertex -> the simplex holding it
        std::vector<size_t> new_vertices;
        std::vector<Rational> along(mp.vertices.size()); // on an edge (a, b), a < b: b's coordinate
        for (size_t i = 0; i < mp.vertices.size(); ++i) {
            const mi::MeshPartition::Vertex& v = mp.vertices[i];
            if (v.simplex.size() == 1) {
                vertex_of[i] = v.simplex[0];
                continue;
            }
            const std::array<Rational, 4> t = mi::exact_barycentric(v, tets, value);
            std::array<Rational, 3> x;
            for (int corner = 0; corner < 4; ++corner) {
                if (t[corner].get_sign() == 0) {
                    continue;
                }
                const Vector3r q = pos(tets[v.tet][corner]);
                for (int k = 0; k < 3; ++k) {
                    x[k] = x[k] + t[corner] * q[k];
                }
            }
            vertex_of[i] = tight_seal_new_vertex(Vector3r(x[0], x[1], x[2]), v.simplex);
            carrier[vertex_of[i]] = v.simplex;
            new_vertices.push_back(vertex_of[i]);
            if (v.simplex.size() == 2) {
                const auto& corners = tets[v.tet];
                along[i] =
                    t[std::find(corners.begin(), corners.end(), v.simplex[1]) - corners.begin()];
            }
        }
        const size_t n_diagram_here = new_vertices.size();
        n_diagram_vertices += n_diagram_here;

        // the faces of the component's tets (local tet, local face: the corner opposite), the
        // cells of each tet, the diagram's vertices on mesh edges in order from the lower id, and
        // the mesh faces with diagram vertices inside
        std::map<Face3, std::vector<std::pair<size_t, int>>> comp_faces;
        for (size_t i = 0; i < tets.size(); ++i) {
            for (int k = 0; k < 4; ++k) {
                comp_faces
                    [sorted3(tets[i][(k + 1) % 4], tets[i][(k + 2) % 4], tets[i][(k + 3) % 4])]
                        .emplace_back(i, k);
            }
        }
        std::vector<std::vector<size_t>> cells_of(tets.size());
        for (size_t ci = 0; ci < mp.cells.size(); ++ci) {
            cells_of[mp.cells[ci].tet].push_back(ci);
        }
        std::map<std::pair<size_t, size_t>, std::vector<size_t>> on_edge;
        std::set<Face3> inner_points;
        for (size_t i = 0; i < mp.vertices.size(); ++i) {
            const auto& s = mp.vertices[i].simplex;
            if (s.size() == 2) {
                on_edge[{s[0], s[1]}].push_back(i);
            } else if (s.size() == 3) {
                inner_points.insert(sorted3(s[0], s[1], s[2]));
            }
        }
        for (auto& [e, points] : on_edge) {
            std::sort(points.begin(), points.end(), [&](const size_t a, const size_t b) {
                return along[a] < along[b];
            });
        }

        // the tets outside the component holding a subdivided edge, or a face with diagram
        // vertices inside
        const std::set<size_t> in_comp(comp_tids.begin(), comp_tids.end());
        const auto tets_on = [&](const std::vector<size_t>& vs) {
            const auto ring = [&](const size_t v) {
                std::vector<size_t> tids = get_one_ring_tids_for_vertex(v);
                std::sort(tids.begin(), tids.end());
                return tids;
            };
            std::vector<size_t> common = ring(vs[0]);
            for (size_t i = 1; i < vs.size(); ++i) {
                common = wmtk::set_intersection(common, ring(vs[i]));
            }
            return common;
        };
        std::set<size_t> beside;
        for (const auto& [e, points] : on_edge) {
            for (const size_t tid : tets_on({e.first, e.second})) {
                if (in_comp.count(tid) == 0) {
                    beside.insert(tid);
                }
            }
        }
        for (const Face3& f : inner_points) {
            for (const size_t tid : tets_on({f[0], f[1], f[2]})) {
                if (in_comp.count(tid) == 0) {
                    beside.insert(tid);
                }
            }
        }
        n_beside += beside.size();

        // A face's triangles, the same seen from either side: the diagram's polygons on a face of
        // the component, each fanned from its smallest diagram vertex id as tetrahedralize does;
        // any other face, fanned from its corner opposite its one subdivided edge, or from a new
        // vertex at its centroid if more of its edges are
        std::map<Face3, size_t> face_centroid;
        const auto face_triangles = [&](const size_t a, const size_t b, const size_t cc) {
            std::vector<std::array<size_t, 3>> out;
            const Face3 f = sorted3(a, b, cc);
            const auto in_comp_face = comp_faces.find(f);
            if (in_comp_face != comp_faces.end()) {
                const auto [i, k] = in_comp_face->second.front();
                for (const size_t ci : cells_of[i]) {
                    for (const auto& face : mp.cells[ci].faces) {
                        if (face.tet_face != k) {
                            continue;
                        }
                        const auto& poly = face.vertices;
                        const size_t n = poly.size();
                        const size_t s = std::min_element(poly.begin(), poly.end()) - poly.begin();
                        for (size_t j = 1; j + 1 < n; ++j) {
                            out.push_back(
                                {{vertex_of[poly[s]],
                                  vertex_of[poly[(s + j) % n]],
                                  vertex_of[poly[(s + j + 1) % n]]}});
                        }
                    }
                }
                return out;
            }
            const std::array<size_t, 3> corner{{a, b, cc}};
            std::vector<size_t> poly;
            int n_cut = 0;
            int cut = -1;
            for (int e = 0; e < 3; ++e) {
                const size_t u = corner[e];
                const size_t w = corner[(e + 1) % 3];
                poly.push_back(u);
                const auto it = on_edge.find({std::min(u, w), std::max(u, w)});
                if (it == on_edge.end()) {
                    continue;
                }
                ++n_cut;
                cut = e;
                std::vector<size_t> points;
                for (const size_t q : it->second) {
                    points.push_back(vertex_of[q]);
                }
                if (u > w) {
                    std::reverse(points.begin(), points.end());
                }
                poly.insert(poly.end(), points.begin(), points.end());
            }
            if (n_cut == 0) {
                out.push_back(corner);
            } else if (n_cut == 1) {
                const size_t apex = corner[(cut + 2) % 3];
                std::rotate(poly.begin(), std::find(poly.begin(), poly.end(), apex), poly.end());
                for (size_t j = 1; j + 1 < poly.size(); ++j) {
                    out.push_back({{poly[0], poly[j], poly[j + 1]}});
                }
            } else {
                auto it = face_centroid.find(f);
                if (it == face_centroid.end()) {
                    const std::vector<size_t> on{f[0], f[1], f[2]};
                    const size_t g = tight_seal_new_vertex(centroid(on), on);
                    carrier[g] = on;
                    new_vertices.push_back(g);
                    ++n_centroids;
                    it = face_centroid.emplace(f, g).first;
                }
                for (size_t j = 0; j < poly.size(); ++j) {
                    out.push_back({{it->second, poly[j], poly[(j + 1) % poly.size()]}});
                }
            }
            return out;
        };

        // the new tets: positively oriented, never flat
        std::vector<std::array<size_t, 4>> new_tets;
        std::vector<TetAttributes> new_attributes;
        std::vector<char> new_filled;
        std::vector<CellTag> new_before;
        std::vector<int64_t> new_comp;
        const auto add_tet = [&](std::array<size_t, 4> t,
                                 const TetAttributes& attributes,
                                 const bool is_filled,
                                 const CellTag& before,
                                 const int64_t comp) {
            const int o = orientation(t);
            if (o == 0) {
                log_and_throw_error("tight_seal_geom: the material interface fill made a flat tet");
            }
            if (o < 0) {
                std::swap(t[2], t[3]);
            }
            new_tets.push_back(t);
            new_attributes.push_back(attributes);
            new_filled.push_back(is_filled ? 1 : 0);
            new_before.push_back(before);
            new_comp.push_back(comp);
        };
        // the component: its cells' tets, each with its color's fill tags added to its tet's
        for (const mi::LabelledTet& lt : mi::tetrahedralize(mp)) {
            const size_t parent = comp_tids[lt.tet];
            TetAttributes attributes = m_tet_attribute[parent];
            const CellTag before = attributes.tags;
            const CellTag& add = fill_tags[colors[lt.function]];
            attributes.tags.insert(add.begin(), add.end());
            add_tet(
                {{vertex_of[lt.vertices[0]],
                  vertex_of[lt.vertices[1]],
                  vertex_of[lt.vertices[2]],
                  vertex_of[lt.vertices[3]]}},
                attributes,
                true,
                before,
                -1);
        }
        std::vector<size_t> old_tets = comp_tids;
        // the tets beside it: coned from a corner whose three faces are fans from it, else from a
        // new vertex at the centroid
        for (const size_t tid : beside) {
            const std::array<size_t, 4> vs = oriented_tet_vids(tid);
            std::array<std::vector<std::array<size_t, 3>>, 4> triangles;
            for (int k = 0; k < 4; ++k) {
                triangles[k] = face_triangles(vs[(k + 1) % 4], vs[(k + 2) % 4], vs[(k + 3) % 4]);
            }
            int apex = -1;
            for (int k = 0; k < 4 && apex < 0; ++k) {
                bool fans = true;
                for (int j = 0; j < 4 && fans; ++j) {
                    if (j == k) {
                        continue;
                    }
                    for (const auto& tri : triangles[j]) {
                        fans = fans && std::find(tri.begin(), tri.end(), vs[k]) != tri.end();
                    }
                }
                if (fans) {
                    apex = k;
                }
            }
            const TetAttributes attributes = m_tet_attribute[tid];
            const int64_t comp = tid < comp_of.size() ? comp_of[tid] : -1;
            if (apex >= 0) {
                for (const auto& tri : triangles[apex]) {
                    add_tet({{vs[apex], tri[0], tri[1], tri[2]}}, attributes, false, {}, comp);
                }
            } else {
                const std::vector<size_t> corners(vs.begin(), vs.end());
                const size_t g = tight_seal_new_vertex(centroid(corners), corners);
                carrier[g] = corners;
                new_vertices.push_back(g);
                ++n_centroids;
                for (int k = 0; k < 4; ++k) {
                    for (const auto& tri : triangles[k]) {
                        add_tet({{g, tri[0], tri[1], tri[2]}}, attributes, false, {}, comp);
                    }
                }
            }
            old_tets.push_back(tid);
        }

        // the vertices of the smallest old simplex holding new vertices: their simplices' union
        const auto span = [&](const std::vector<size_t>& vs) {
            std::vector<size_t> spanned;
            for (const size_t v : vs) {
                const auto it = carrier.find(v);
                if (it == carrier.end()) {
                    spanned.push_back(v);
                } else {
                    spanned.insert(spanned.end(), it->second.begin(), it->second.end());
                }
            }
            vector_unique(spanned);
            return spanned;
        };
        const std::vector<size_t> ids =
            tight_seal_replace_tets(old_tets, new_tets, new_attributes, span);

        filled.resize(tet_capacity(), 0);
        tags_before.resize(tet_capacity());
        comp_of.resize(tet_capacity(), -1);
        for (size_t i = ids.size(); i < old_tets.size(); ++i) {
            comp_of[old_tets[i]] = -1;
        }
        for (size_t i = 0; i < ids.size(); ++i) {
            filled[ids[i]] = new_filled[i];
            comp_of[ids[i]] = new_comp[i];
            if (new_filled[i]) {
                tags_before[ids[i]] = new_before[i];
                ++n_filled;
            }
        }
        for (const size_t v : new_vertices) {
            n_rounded += round(tuple_from_vertex(v)) ? 1 : 0;
        }
        n_new_vertices += new_vertices.size();
        logger().info(
            "tight_seal_geom: material interface fill of component {}: {} tets, {} colors, {} "
            "cells; {} new vertices on the diagram; {} tets beside it cut to match",
            c,
            comp_tids.size(),
            colors.size(),
            mp.cells.size(),
            n_diagram_here,
            beside.size());
    }
    logger().info(
        "tight_seal_geom: material interface fill: {} cells in {} components, {} tets filled; {} "
        "new vertices ({} on the diagram, {} at the centroid of a face or a tet beside a part, for "
        "the {} tets cut to match), {} of them rounded",
        n_cells,
        components.size(),
        n_filled,
        n_new_vertices,
        n_diagram_vertices,
        n_centroids,
        n_beside,
        n_rounded);
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
}

void SimWildMesh::tight_seal_topology_check(
    const std::vector<char>& filled,
    const std::vector<CellTag>& tags_before) const
{
    std::vector<size_t> tids;
    for (const Tuple& t : get_tets()) {
        tids.push_back(t.tid(*this));
    }
    tight_seal::topology_check(
        tids,
        [&](const size_t tid) -> const CellTag& { return m_tet_attribute[tid].tags; },
        filled,
        tags_before,
        [&](const size_t tid) {
            std::vector<size_t> nbrs;
            for (int j = 0; j < 4; ++j) {
                if (const auto opp = tuple_from_face(tid, j).switch_tetrahedron(*this)) {
                    nbrs.push_back(opp->tid(*this));
                }
            }
            return nbrs;
        },
        [&](const std::vector<size_t>& cells, const wmtk::utils::CellTest& in) {
            return wmtk::utils::nonmanifold_vertices(*this, cells, in).size();
        },
        m_tag_id_to_name,
        "tets");
}

void SimWildMesh::tight_seal_geom_sanity(
    const std::vector<CellTag>& fill_tags,
    const std::function<int(size_t)>& color_of,
    const std::vector<char>& filled) const
{
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
        const bool holds_a_set =
            std::any_of(fill_tags.begin(), fill_tags.end(), [&](const CellTag& set) {
                return std::includes(tag.begin(), tag.end(), set.begin(), set.end());
            });
        if (!holds_a_set) {
            log_and_throw_error(
                "tight_seal_geom sanity check: filled tet {} has tags {}, holding none of the "
                "fill tag sets",
                tid,
                std::vector<int64_t>(tag.begin(), tag.end()));
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
        std::set<int> borders;
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
                const int k = color_of(nbr);
                if (k >= 0) {
                    borders.insert(k);
                }
            }
        }
        ++n_parts;
        if (borders.size() < 2) {
            log_and_throw_error(
                "tight_seal_geom sanity check: a filled part of {} tets borders only the "
                "colors {}, so a concave feature of one body was filled",
                part.size(),
                std::vector<int>(borders.begin(), borders.end()));
        }
    }
    logger().info(
        "tight_seal_geom sanity check passed: no tet inverted; {} filled tets in {} "
        "parts, each bordering two colors or more",
        filled_ids.size(),
        n_parts);
}

} // namespace wmtk::components::simwild
