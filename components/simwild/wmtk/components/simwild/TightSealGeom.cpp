#include "TightSealGeom.hpp"

#include "MaterialInterface.hpp"
#include "SimWildMesh.h"
#include "SimWildMeshTri.hpp"
#include "TightSealPairFill.hpp"

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
    std::vector<std::pair<size_t, size_t>>* children_of)
{
    const auto [e0, e1] = e.vertices();
    const auto [t, eid] = tuple_from_edge(e.vertices());

    // Each face on the edge gives two children, which keep the face's vertex opposite the
    // edge. Record by that vertex the face's id, whether it is in the set and whether it was
    // filled.
    const std::vector<size_t> parents = get_incident_fids_for_edge(t);
    std::map<size_t, std::tuple<size_t, bool, char>> parent_state;
    for (const size_t fid : parents) {
        parent_state[simplex_from_face(fid).opposite_vertex(e).id()] = {
            fid,
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
    if (children_of) {
        children_of->clear();
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
        const auto& [parent, in_set, parent_filled] = parent_state.at(opp_vid);
        filled[fid] = parent_filled;
        if (children_of) {
            children_of->emplace_back(parent, fid);
        }
        if (!in_set) {
            continue; // outside the set, keeps its tags
        }
        faces.insert(fid);
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

void SimWildMeshTri::tight_seal_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_colors,
    const std::vector<CellTag>& fill_tags,
    const std::vector<SimpleBVH::BVH>& color_bvhs,
    std::vector<char>& filled,
    std::vector<CellTag>& tags_before)
{
    std::vector<std::vector<size_t>> comp_cells(components.size());
    std::vector<size_t> n_functions(components.size());
    std::unordered_set<size_t> cells; // every face of the components, kept through the splits
    for (size_t c = 0; c < components.size(); ++c) {
        comp_cells[c] = components[c].cells;
        std::sort(comp_cells[c].begin(), comp_cells[c].end());
        n_functions[c] = components_colors[c].size();
        cells.insert(comp_cells[c].begin(), comp_cells[c].end());
    }

    // Component c's function k: the distance to its color k. Each split goes exactly where the
    // pair's difference, linear along the edge, vanishes: delta_a / (delta_a - delta_b) of the
    // way from a, exact for the doubles given (split_adjust_position).
    tight_seal::ClearOnExit<std::optional<EdgeSplitAt>> clear_split_at{m_edge_split_at};
    tight_seal::PairFill<3>::Mesh mesh;
    mesh.corners = [&](const size_t id) { return oriented_tri_vids(id); };
    mesh.value = [&](const size_t c, const size_t k, const size_t v) {
        return tight_seal::bvh_distance(
            color_bvhs[components_colors[c][k]],
            m_vertex_attribute[v].m_posf);
    };
    mesh.split = [&](const size_t a,
                     const size_t b,
                     const double delta_a,
                     const double delta_b,
                     std::vector<std::pair<size_t, size_t>>& children) {
        const Rational ra(delta_a);
        m_edge_split_at = EdgeSplitAt{a, ra / (ra - Rational(delta_b))};
        const size_t v = tight_seal_split_tracked(simplex::Edge(a, b), cells, filled, &children);
        m_edge_split_at.reset();
        return v;
    };
    tight_seal::PairFill<3> fill(mesh, comp_cells, n_functions);
    fill.run();

    // Each face of the components gets its label's color's fill tags added to its own.
    tags_before.resize(tri_capacity());
    filled.resize(tri_capacity(), 0);
    size_t n_filled = 0;
    for (const Tuple& t : get_faces()) {
        const size_t id = t.fid(*this);
        const int c = fill.component(id);
        if (c < 0) {
            continue;
        }
        const int label = fill.label(id);
        if (label < 0) {
            log_and_throw_error("tight_seal_geom: the fill left a face of a component unlabelled");
        }
        const size_t color = components_colors[size_t(c)][size_t(label)];
        CellTag& tags = m_face_attribute[id].tags;
        tags_before[id] = tags;
        tags.insert(fill_tags[color].begin(), fill_tags[color].end());
        filled[id] = 1;
        ++n_filled;
    }

    // The vertices left exact (their double point inverted a face), rounded where that inverts
    // none now.
    size_t n_exact = 0, n_rounded = 0;
    for (const size_t v : fill.new_vertices()) {
        if (m_vertex_attribute[v].m_is_rounded) {
            continue;
        }
        ++n_exact;
        if (round(tuple_from_vertex(v))) {
            ++n_rounded;
            for (const size_t id : get_one_ring_fids_for_vertex(v)) {
                m_face_attribute[id].m_quality = get_quality(tuple_from_tri(id));
            }
        }
    }

    const auto& st = fill.stats();
    logger().info(
        "tight_seal_geom: fill of {} components, {} faces ({} with a color common to their "
        "corners from the start): cut along {} pairs of colors ({} colors left out as dominated), "
        "{} edges split ({} refused); the new vertices tie {} more pairs of colors than the pair "
        "each is made for, through ties or to stay on their edge's side of a level set; {} new "
        "vertices exact, {} of them then rounded",
        components.size(),
        st.n_original,
        st.n_original_finalized,
        st.n_pairs,
        st.n_dominated,
        st.n_split,
        st.n_refused,
        st.n_closed,
        n_exact,
        n_rounded);
    logger().info(
        "tight_seal_geom: fill labelled {} faces: {} by the color their corners share, {} by the "
        "color nearest their centroid",
        n_filled,
        st.n_cells - st.n_fallback,
        st.n_fallback);
    if (st.n_refused > 0 || st.n_fallback > 0) {
        logger().warn(
            "tight_seal_geom: fill: {} splits refused; {} faces with no color common to their "
            "corners, labelled by the color nearest their centroid",
            st.n_refused,
            st.n_fallback);
    }
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
    std::vector<std::pair<size_t, size_t>>* children_of)
{
    const auto [e0, e1] = e.vertices();
    const Tuple t = tuple_from_edge(e.vertices());

    // Each tet on the edge gives two children, which keep the tet's edge opposite the split
    // edge. Record by that edge the tet's id, whether it is in the set and whether it was filled.
    const std::vector<size_t> parents = get_incident_tids_for_edge(t);
    std::map<simplex::Edge, std::tuple<size_t, bool, char>> parent_state;
    for (const size_t tid : parents) {
        parent_state[simplex_from_tet(tid).opposite_edge(e)] = {
            tid,
            cells.count(tid) > 0,
            filled[tid]};
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
    if (children_of) {
        children_of->clear();
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
        const auto& [parent, in_set, parent_filled] = it->second;
        filled[tid] = parent_filled;
        if (children_of) {
            children_of->emplace_back(parent, tid);
        }
        if (!in_set) {
            continue; // outside the set, keeps its tags
        }
        cells.insert(tid);
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

void SimWildMesh::tight_seal_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_colors,
    const std::vector<CellTag>& fill_tags,
    const std::vector<SimpleBVH::BVH>& color_bvhs,
    std::vector<char>& filled,
    std::vector<CellTag>& tags_before)
{
    std::vector<std::vector<size_t>> comp_cells(components.size());
    std::vector<size_t> n_functions(components.size());
    std::unordered_set<size_t> cells; // every tet of the components, kept through the splits
    for (size_t c = 0; c < components.size(); ++c) {
        comp_cells[c] = components[c].cells;
        std::sort(comp_cells[c].begin(), comp_cells[c].end());
        n_functions[c] = components_colors[c].size();
        cells.insert(comp_cells[c].begin(), comp_cells[c].end());
    }

    // Component c's function k: the distance to its color k. Each split goes exactly where the
    // pair's difference, linear along the edge, vanishes: delta_a / (delta_a - delta_b) of the
    // way from a, exact for the doubles given (split_adjust_position).
    tight_seal::ClearOnExit<std::optional<EdgeSplitAt>> clear_split_at{m_edge_split_at};
    tight_seal::PairFill<4>::Mesh mesh;
    mesh.corners = [&](const size_t id) { return oriented_tet_vids(id); };
    mesh.value = [&](const size_t c, const size_t k, const size_t v) {
        return tight_seal::bvh_distance(
            color_bvhs[components_colors[c][k]],
            m_vertex_attribute[v].m_posf);
    };
    mesh.split = [&](const size_t a,
                     const size_t b,
                     const double delta_a,
                     const double delta_b,
                     std::vector<std::pair<size_t, size_t>>& children) {
        const Rational ra(delta_a);
        m_edge_split_at = EdgeSplitAt{a, ra / (ra - Rational(delta_b))};
        const size_t v = tight_seal_split_tracked(simplex::Edge(a, b), cells, filled, &children);
        m_edge_split_at.reset();
        return v;
    };
    tight_seal::PairFill<4> fill(mesh, comp_cells, n_functions);
    fill.run();

    // Each tet of the components gets its label's color's fill tags added to its own.
    tags_before.resize(tet_capacity());
    filled.resize(tet_capacity(), 0);
    size_t n_filled = 0;
    for (const Tuple& t : get_tets()) {
        const size_t id = t.tid(*this);
        const int c = fill.component(id);
        if (c < 0) {
            continue;
        }
        const int label = fill.label(id);
        if (label < 0) {
            log_and_throw_error("tight_seal_geom: the fill left a tet of a component unlabelled");
        }
        const size_t color = components_colors[size_t(c)][size_t(label)];
        CellTag& tags = m_tet_attribute[id].tags;
        tags_before[id] = tags;
        tags.insert(fill_tags[color].begin(), fill_tags[color].end());
        filled[id] = 1;
        ++n_filled;
    }

    // The vertices left exact (their double point inverted a tet), rounded where that inverts
    // none now.
    size_t n_exact = 0, n_rounded = 0;
    for (const size_t v : fill.new_vertices()) {
        if (m_vertex_attribute[v].m_is_rounded) {
            continue;
        }
        ++n_exact;
        if (round(tuple_from_vertex(v))) {
            ++n_rounded;
            for (const size_t id : get_one_ring_tids_for_vertex(v)) {
                m_tet_attribute[id].m_quality = get_quality(tuple_from_tet(id));
            }
        }
    }

    const auto& st = fill.stats();
    logger().info(
        "tight_seal_geom: fill of {} components, {} tets ({} with a color common to their "
        "corners from the start): cut along {} pairs of colors ({} colors left out as dominated), "
        "{} edges split ({} refused); the new vertices tie {} more pairs of colors than the pair "
        "each is made for, through ties or to stay on their edge's side of a level set; {} new "
        "vertices exact, {} of them then rounded",
        components.size(),
        st.n_original,
        st.n_original_finalized,
        st.n_pairs,
        st.n_dominated,
        st.n_split,
        st.n_refused,
        st.n_closed,
        n_exact,
        n_rounded);
    logger().info(
        "tight_seal_geom: fill labelled {} tets: {} by the color their corners share, {} by the "
        "color nearest their centroid",
        n_filled,
        st.n_cells - st.n_fallback,
        st.n_fallback);
    if (st.n_refused > 0 || st.n_fallback > 0) {
        logger().warn(
            "tight_seal_geom: fill: {} splits refused; {} tets with no color common to their "
            "corners, labelled by the color nearest their centroid",
            st.n_refused,
            st.n_fallback);
    }
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
}

size_t SimWildMesh::tight_seal_new_vertex(
    const Vector3r& position,
    const std::vector<size_t>& carrier)
{
    // Its surface status and order from the simplex it lies in, as a split there sets them
    // (split_edge, split_face, split_tet): read now, while that simplex is still in the mesh.
    bool on_surface = false;
    size_t order = 0;
    if (carrier.size() == 2) {
        on_surface = is_edge_on_surface(tuple_from_edge({{carrier[0], carrier[1]}}));
        order = on_surface ? get_order_of_edge({{carrier[0], carrier[1]}}) : 0;
    } else if (carrier.size() == 3) {
        std::array<size_t, 3> f{{carrier[0], carrier[1], carrier[2]}};
        std::sort(f.begin(), f.end());
        on_surface = m_face_attribute[std::get<1>(tuple_from_face(f))].m_is_surface_fs;
        order = on_surface ? 1 : 0;
    }

    const size_t v = add_vertex_slot();
    VertexAttributes va(position);
    va.m_is_rounded = false;
    va.m_is_on_surface = on_surface;
    va.m_order = order;
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
