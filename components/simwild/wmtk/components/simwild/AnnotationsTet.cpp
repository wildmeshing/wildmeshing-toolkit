#include "MultiVoronoiEnumeration.hpp"
#include "SimWildMesh.h"
#include "TightSealGeom.hpp"

#include <wmtk/utils/SimplicialEmbedding.hpp>

#include <unordered_set>

namespace wmtk::components::simwild {

std::vector<ConnectedComponent> SimWildMesh::compute_connected_components(
    const CellTag& tag_in) const
{
    std::vector<int> comp_id(tet_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (comp_id[tid] != -1) {
            continue;
        }

        const CellTag& tag = m_tet_attribute[tid].tags;

        // check if face contains the tag_in tags
        if (!set_includes(tag, tag_in)) {
            continue;
        }

        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();

        comp.cells.push_back(tid);
        comp_id[tid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += tet_volume(cur);
            for (int j = 0; j < 4; ++j) {
                const Tuple edge_tup = tuple_from_face(cur, j);
                const auto t_opp = edge_tup.switch_tetrahedron(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->tid(*this);
                if (comp_id[nbr] != -1) {
                    // components should never connect to other components
                    if (comp_id[nbr] != comp_idx) {
                        log_and_throw_error(
                            "Components {} and {} are neighboring.",
                            comp_idx,
                            comp_id[nbr]);
                    }
                    // already in a component
                    continue;
                }
                const CellTag& ntag = m_tet_attribute[nbr].tags;
                if (!set_includes(ntag, tag_in)) {
                    continue;
                }
                comp_id[nbr] = comp_idx;
                comp.cells.push_back(nbr);
            }
        }
    }

    return components;
}

std::vector<ConnectedComponent> SimWildMesh::compute_connected_components(const ExprPtr& expr) const
{
    std::vector<int> comp_id(tet_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (comp_id[tid] != -1) {
            continue;
        }

        const CellTag& tag = m_tet_attribute[tid].tags;

        // check if face contains the tag_in tags
        if (!expr->eval(tag)) {
            continue;
        }

        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();

        comp.cells.push_back(tid);
        comp_id[tid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += tet_volume(cur);
            for (int j = 0; j < 4; ++j) {
                const Tuple edge_tup = tuple_from_face(cur, j);
                const auto t_opp = edge_tup.switch_tetrahedron(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->tid(*this);
                if (comp_id[nbr] != -1) {
                    // components should never connect to other components
                    if (comp_id[nbr] != comp_idx) {
                        log_and_throw_error(
                            "Components {} and {} are neighboring.",
                            comp_idx,
                            comp_id[nbr]);
                    }
                    // already in a component
                    continue;
                }
                const CellTag& ntag = m_tet_attribute[nbr].tags;
                if (!expr->eval(ntag)) {
                    continue;
                }
                comp_id[nbr] = comp_idx;
                comp.cells.push_back(nbr);
            }
        }
    }

    return components;
}


std::vector<ConnectedComponent> SimWildMesh::find_holes(const std::vector<CellTag>& tag_in) const
{
    std::vector<int> comp_id(tet_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (comp_id[tid] != -1) {
            continue;
        }

        const CellTag& tag = m_tet_attribute[tid].tags;

        // check if face contains one of the tag_in tags
        {
            bool is_hole = true;
            for (const CellTag& ti : tag_in) {
                if (set_includes(tag, ti)) {
                    is_hole = false;
                    break;
                }
            }
            if (!is_hole) {
                continue;
            }
        }

        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();

        comp.cells.push_back(tid);
        comp_id[tid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += tet_volume(cur);
            for (int j = 0; j < 4; ++j) {
                const Tuple edge_tup = tuple_from_face(cur, j);
                const auto t_opp = edge_tup.switch_tetrahedron(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->tid(*this);
                if (comp_id[nbr] != -1) {
                    // holes should never connect to other holes
                    if (comp_id[nbr] != comp_idx) {
                        log_and_throw_error(
                            "Hole componenents {} and {} are neighboring.",
                            comp_idx,
                            comp_id[nbr]);
                    }
                    // already in a component
                    continue;
                }
                const CellTag& ntag = m_tet_attribute[nbr].tags;
                bool n_in_hole = true;
                for (const CellTag& ti : tag_in) {
                    if (set_includes(ntag, ti)) {
                        n_in_hole = false;
                        break;
                    }
                }
                if (!n_in_hole) {
                    continue;
                }
                comp_id[nbr] = comp_idx;
                comp.cells.push_back(nbr);
            }
        }
    }

    return components;
}

void SimWildMesh::compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& F) const
{
    // extract boundary edges
    std::vector<Vector3i> faces;
    std::vector<Vector3d> vertices;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!set_includes(m_tet_attribute[tid].tags, tag)) {
            continue;
        }

        for (int j = 0; j < 4; ++j) {
            const Tuple edge_tup = tuple_from_face(tid, j);
            const auto t_opp = edge_tup.switch_tetrahedron(*this);
            if (!t_opp) {
                continue; // boundary
            }
            const size_t nbr = t_opp->tid(*this);
            const CellTag& ntag = m_tet_attribute[nbr].tags;
            if (!set_includes(ntag, tag)) {
                // found a component boundary, add to BVH
                size_t v0 = edge_tup.vid(*this);
                size_t v1 = edge_tup.switch_vertex(*this).vid(*this);
                size_t v2 = edge_tup.switch_edge(*this).switch_vertex(*this).vid(*this);
                size_t nv = vertices.size();
                faces.emplace_back(nv, nv + 1, nv + 2);
                vertices.push_back(m_vertex_attribute.at(v0).m_posf);
                vertices.push_back(m_vertex_attribute.at(v1).m_posf);
                vertices.push_back(m_vertex_attribute.at(v2).m_posf);
            }
        }
    }

    assert(3 * faces.size() == vertices.size());

    V.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        V.row(i) = vertices[i];
    }
    F.resize(faces.size(), 3);
    for (size_t i = 0; i < faces.size(); ++i) {
        F.row(i) = faces[i];
    }
}

void SimWildMesh::fill_holes_topo(const std::vector<CellTag>& fill_holes_tags, double threshold)
{
    if (m_tags_count == 0) {
        logger().warn("fill_holes_topo: no tags, skipping");
        return;
    }

    for (const CellTag& fill_tag : fill_holes_tags) {
        const std::vector<ConnectedComponent> holes = find_holes({fill_tag});

        size_t fill_counter = 0;
        size_t thre_counter = 0;
        size_t bndy_counter = 0;
        for (const ConnectedComponent& comp : holes) {
            if (comp.touches_boundary) {
                ++bndy_counter;
                continue;
            }

            if (threshold >= 0 && comp.volume > threshold) {
                ++thre_counter;
                continue; // only consider holes smaller than the threshold
            }

            ++fill_counter;
            for (const size_t tid : comp.cells) {
                auto& tag = m_tet_attribute[tid].tags;
                tag.insert(fill_tag.begin(), fill_tag.end());
            }
        }

        // sanity check
        if (fill_counter + thre_counter + bndy_counter != holes.size()) {
            log_and_throw_error(
                "#holes = {}, #filled = {}, #on_boundary = {}, #too_small = {}",
                holes.size(),
                fill_counter,
                bndy_counter,
                thre_counter);
        }

        logger().info("{} holes filled for tags {}", fill_counter, fill_tag);
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::seal_connected_components(
    const std::vector<CellTag>& tag_set,
    const std::vector<ConnectedComponent>& components)
{
    auto get_center = [&](const size_t fid) {
        const auto vs = oriented_tet_vids(fid);
        const Vector3d& p0 = m_vertex_attribute[vs[0]].m_posf;
        const Vector3d& p1 = m_vertex_attribute[vs[1]].m_posf;
        const Vector3d& p2 = m_vertex_attribute[vs[2]].m_posf;
        const Vector3d& p3 = m_vertex_attribute[vs[3]].m_posf;
        return (p0 + p1 + p2 + p3) / 4;
    };

    // BVH for each tag_set
    std::vector<SimpleBVH::BVH> bvhs;
    for (const auto& tag : tag_set) {
        MatrixXd V;
        MatrixXi F;
        compute_tag_boundary(tag, V, F);
        auto& bvh = bvhs.emplace_back();
        bvh.init(V, F, 0);
    }

    // build SDF
    m_voronoi_split_fn = [&](const Vector3d& p) -> double {
        double sq_dist;
        Vector3d nearest_point;

        bvhs[0].nearest_facet(p, nearest_point, sq_dist);
        const double da = std::sqrt(sq_dist);

        bvhs[1].nearest_facet(p, nearest_point, sq_dist);
        const double db = std::sqrt(sq_dist);

        return da - db;
    };
    struct ClearVoronoiSplit
    {
        std::function<double(const Vector3d&)>& fn;
        ~ClearVoronoiSplit() { fn = nullptr; }
    } clear_voronoi_split{m_voronoi_split_fn};

    // seal holes
    for (const ConnectedComponent& hole : components) {
        std::vector<simplex::Edge> split_edges;
        std::unordered_set<size_t> vids;
        std::unordered_set<size_t> vids_left;
        for (const size_t tid : hole.cells) {
            const Vector3d p = get_center(tid);
            const double d = m_voronoi_split_fn(p);
            auto& tag = m_tet_attribute[tid].tags;
            if (d < 0) {
                tag.insert(tag_set[0].begin(), tag_set[0].end());
            } else {
                tag.insert(tag_set[1].begin(), tag_set[1].end());
            }

            for (int j = 0; j < 6; ++j) {
                const Tuple t = tuple_from_edge(tid, j);
                const size_t v0 = t.vid(*this);
                const size_t v1 = t.switch_vertex(*this).vid(*this);
                vids.insert(v0);
                vids.insert(v1);
                const double d0 = m_voronoi_split_fn(m_vertex_attribute.at(v0).m_posf);
                const double d1 = m_voronoi_split_fn(m_vertex_attribute.at(v1).m_posf);
                // only split edges if their endpoints aren't already on the surface
                if ((d0 < -1e-20 && d1 > 1e-20) || (d1 < -1e-20 && d0 > 1e-20)) {
                    split_edges.emplace_back(v0, v1);
                }
            }
        }
        for (const size_t vid : vids) {
            const double d = m_voronoi_split_fn(m_vertex_attribute.at(vid).m_posf);
            if (d < 0) {
                vids_left.insert(vid);
            }
        }
        vector_unique(split_edges);

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }

        std::vector<Tuple> new_edges;
        std::unordered_set<size_t> new_vertices;
        for (const simplex::Edge& e : split_edges) {
            const Tuple t = tuple_from_edge(e.vertices());

            reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
            if (!split_edge(t, new_edges)) {
                continue;
            }

            const size_t v_new = split_tag_cache.local().v_new;
            std::vector<size_t> tids = get_one_ring_tids_for_vertex(v_new);

            new_vertices.insert(v_new);

            for (const size_t tid : tids) {
                const auto vs = oriented_tet_vids(tid);
                // find vertex from splitted edge
                size_t vid = -1;
                bool is_inside = true;
                for (const size_t v : vs) {
                    if (v == v_new) {
                        continue;
                    }
                    if (e.vertices()[0] == v || e.vertices()[1] == v) {
                        vid = v;
                    } else if (vids.count(v) == 0 && new_vertices.count(v) == 0) {
                        // check the two other vertices if they are hole vertices
                        is_inside = false;
                        break;
                    }
                }

                if (!is_inside) {
                    // not inside the hole, do not change tags
                    continue;
                }

                if (vid == -1) {
                    log_and_throw_error("Could not find edge-vertex after split.");
                }

                auto& tag = m_tet_attribute[tid].tags;
                if (vids_left.count(vid) > 0) {
                    for (const size_t tt : tag_set[1]) {
                        tag.erase(tt);
                    }
                    tag.insert(tag_set[0].begin(), tag_set[0].end());
                } else {
                    for (const size_t tt : tag_set[0]) {
                        tag.erase(tt);
                    }
                    tag.insert(tag_set[1].begin(), tag_set[1].end());
                }
            }

            // if (m_params.debug_output) {
            //     write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            // }
        }

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}

void SimWildMesh::keep_largest_connected_component(
    const std::vector<CellTag>& lcc_tags,
    const size_t n_lcc)
{
    for (const CellTag& lcc_tag : lcc_tags) {
        auto components = compute_connected_components(lcc_tag);
        const size_t n = std::min(components.size(), n_lcc);
        logger().info(
            "Found {} components with tag {}, keeping the {} largest.",
            components.size(),
            lcc_tag,
            n);

        if (components.empty()) {
            continue;
        }

        std::multimap<double, size_t> components_by_volume;
        for (size_t i = 0; i < components.size(); ++i) {
            components_by_volume.insert(std::make_pair(components[i].volume, i));
        }

        size_t counter = 0;
        for (const auto& [_, i] : components_by_volume) {
            if (counter + n >= components.size()) {
                break;
            }
            for (const size_t tid : components[i].cells) {
                auto& tag = m_tet_attribute[tid].tags;
                for (const size_t remove_tag : lcc_tag) {
                    tag.erase(remove_tag);
                }
            }
            ++counter;
        }

        // check result
        components = compute_connected_components(lcc_tag);
        if (components.size() != n) {
            log_and_throw_error(
                "Failed to remove components with tag {}. #components = {}",
                lcc_tag,
                components.size());
        }
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::tight_seal_topo(
    const std::vector<std::vector<CellTag>>& tight_seal_tag_sets,
    double threshold)
{
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }

    for (const std::vector<CellTag>& tag_set : tight_seal_tag_sets) {
        if (tag_set.size() != 2) {
            log_and_throw_error(
                "Can only create a tight seal between two tags. Input contains {} tags. Input was "
                "{}",
                tag_set.size(),
                tag_set);
        }

        const std::vector<ConnectedComponent> holes = find_holes(tag_set);
        // logger().info("Found {} holes in between tags {}", holes.size(), tag_set);

        // find the holes that connect to all tags in tag_set
        std::vector<ConnectedComponent> holes_seal;
        for (const ConnectedComponent& hole : holes) {
            if (hole.touches_boundary) {
                continue;
            }

            if (threshold >= 0 && hole.volume > threshold) {
                continue; // only consider holes smaller than the threshold
            }

            std::vector<bool> found_tag_set(tag_set.size(), false);

            for (const size_t tid : hole.cells) {
                for (int j = 0; j < 4; ++j) {
                    const Tuple edge_tup = tuple_from_face(tid, j);
                    const auto t_opp = edge_tup.switch_tetrahedron(*this);
                    if (!t_opp) {
                        // boundary
                        continue;
                    }
                    const size_t nbr = t_opp->tid(*this);
                    const CellTag& ntag = m_tet_attribute[nbr].tags;
                    for (size_t i = 0; i < tag_set.size(); ++i) {
                        if (set_includes(ntag, tag_set[i])) {
                            found_tag_set[i] = true;
                        }
                    }
                }

                bool found_all = true;
                for (size_t i = 0; i < tag_set.size(); ++i) {
                    if (!found_tag_set[i]) {
                        found_all = false;
                        break;
                    }
                }
                if (found_all) {
                    holes_seal.push_back(hole);
                    break;
                }
            }
        }

        logger().info("Seal {} holes in between tags {}", holes_seal.size(), tag_set);

        if (holes_seal.empty()) {
            continue;
        }

        seal_connected_components(tag_set, holes_seal);
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::tight_seal_geom(
    const std::vector<int64_t>& listed_tags,
    const double distance,
    const double trace_rel_tol,
    const std::string& selection,
    const int64_t min_regions,
    const bool sanity,
    const std::string& mode,
    const ExprPtr& spatial_selection)
{
    const bool split_fill = mode == "split_fill";
    const bool fill_mode = mode == "fill" || split_fill; // neither has the limit of three regions
    if (!fill_mode && mode != "voronoi") {
        log_and_throw_error(
            "tight_seal_geom: mode must be \"voronoi\", \"fill\" or \"split_fill\". Input was {}",
            mode);
    }
    const bool detected = selection == "detected";
    if (!detected && selection != "explicit") {
        log_and_throw_error(
            "tight_seal_geom: selection must be \"explicit\" or \"detected\". Input was {}",
            selection);
    }
    if (detected && (min_regions < 2 || (!fill_mode && min_regions > 3))) {
        log_and_throw_error(
            "tight_seal_geom: min_regions must be {}. Input was {}",
            fill_mode ? "at least 2" : "2 or 3",
            min_regions);
    }
    if (!detected && min_regions != 2) {
        logger().warn(
            "tight_seal_geom: min_regions {} is ignored with selection \"explicit\", which fills "
            "the parts that border every listed tag",
            min_regions);
    }
    // the tags of the regions: the listed ones, or every tag but ambient that some tet has alone
    std::vector<int64_t> tags;
    if (detected) {
        if (!listed_tags.empty()) {
            logger().warn(
                "tight_seal_geom: selection \"detected\" ignores the listed tags {}",
                listed_tags);
        }
        CellTag found;
        for (const Tuple& t : get_tets()) {
            const CellTag& tag = m_tet_attribute[t.tid(*this)].tags;
            if (tag.size() == 1 && !tight_seal::is_ambient(m_tag_id_to_name, *tag.begin())) {
                found.insert(*tag.begin());
            }
        }
        tags.assign(found.begin(), found.end());
    } else {
        tags = listed_tags;
        if (tags.size() < 2 || (!fill_mode && tags.size() > 3)) {
            log_and_throw_error(
                "tight_seal_geom needs {} tags. Input was {}",
                fill_mode ? "two or more" : "two or three",
                tags);
        }
        if (CellTag(tags.begin(), tags.end()).size() != tags.size()) {
            log_and_throw_error("tight_seal_geom needs distinct tags. Input was {}", tags);
        }
        for (const int64_t tag : tags) {
            if (tight_seal::is_ambient(m_tag_id_to_name, tag)) {
                log_and_throw_error("tight_seal_geom cannot seal ambient. Input was {}", tags);
            }
        }
    }
    const size_t n = tags.size();
    // the number of regions a component must border to be filled: min_regions, or all n listed
    const size_t needed = detected ? size_t(min_regions) : n;
    if (!(distance > 0)) {
        log_and_throw_error("tight_seal_geom needs a positive distance. Input was {}", distance);
    }
    if (!(trace_rel_tol > 0 && trace_rel_tol < 1)) {
        log_and_throw_error(
            "tight_seal_geom needs a trace tolerance in (0, 1). Input was {}",
            trace_rel_tol);
    }
    const double radius = distance / 2;

    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }

    const auto pos = [&](const size_t vid) -> const Vector3d& {
        return m_vertex_attribute[vid].m_posf;
    };

    // The region of a tet: i for the tets of R_i (tagged tags[i] and nothing else), -1 for E. By
    // tag, so it holds for the tets the splits below make.
    const auto region_of = [&](const size_t tid) {
        const CellTag& tag = m_tet_attribute[tid].tags;
        if (tag.size() != 1) {
            return -1;
        }
        const auto it = std::find(tags.begin(), tags.end(), *tag.begin());
        return it == tags.end() ? -1 : int(it - tags.begin());
    };
    std::vector<bool> region_found(n, false);
    for (const Tuple& t : get_tets()) {
        const int r = region_of(t.tid(*this));
        if (r >= 0) {
            region_found[r] = true;
        }
    }
    const bool can_seal =
        n >= needed &&
        std::find(region_found.begin(), region_found.end(), false) == region_found.end();

    // 0. R, the union of the regions, is made simplicially embedded
    // (wmtk/utils/SimplicialEmbedding.hpp): each tet outside R whose four faces lie on R is split
    // at its centroid, then each face outside R whose three edges do, then each edge outside R
    // joining two vertices of R at its midpoint. Then no tet outside R has all its corners on R:
    // a gap one tet wide gets a vertex off R, whose distances decide whether the gap is in J.
    // (Every vertex of R counts as inside J, so without the embedding such a tet would be in J at
    // any distance.)
    if (can_seal) {
        const wmtk::utils::TetComplex R_complex{
            [&](const size_t vid) {
                for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
                    if (region_of(tid) >= 0) {
                        return true;
                    }
                }
                return false;
            },
            [&](const Tuple& e) {
                for (const Tuple& t :
                     get_incident_tets_for_edge(e.vid(*this), switch_vertex(e).vid(*this))) {
                    if (region_of(t.tid(*this)) >= 0) {
                        return true;
                    }
                }
                return false;
            },
            [&](const Tuple& f) {
                if (region_of(f.tid(*this)) >= 0) {
                    return true;
                }
                const auto opp = f.switch_tetrahedron(*this);
                return opp.has_value() && region_of(opp->tid(*this)) >= 0;
            },
            [&](const Tuple& t) { return region_of(t.tid(*this)) >= 0; }};
        const wmtk::utils::SimplicialEmbeddingStats emb =
            wmtk::utils::simplicial_embedding(*this, R_complex);
        logger().info(
            "tight_seal_geom: simplicial embedding of the regions: split {} tets, {} faces and {} "
            "edges",
            emb.tets_split,
            emb.faces_split,
            emb.edges_split);
        if (emb.refused > 0) {
            logger().warn(
                "tight_seal_geom: {} splits of the simplicial embedding were refused; a tet "
                "outside the regions may still have all its corners on them",
                emb.refused);
        }
    }

    std::vector<size_t> tids;
    std::vector<int> region(tet_capacity(), -1);
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        tids.push_back(tid);
        region[tid] = region_of(tid);
    }

    // m_voronoi_split_fn places the vertex of each split below (split_adjust_position)
    tight_seal::ClearOnExit<std::function<double(const Vector3d&)>> clear_voronoi_split{
        m_voronoi_split_fn};

    // filled[tid]: the Voronoi fill set the tet's region. A split's children inherit it from
    // their parent, so it stays right whichever component's split cuts the tet. For the sanity
    // check.
    std::vector<char> filled(tet_capacity(), 0);

    std::vector<SimpleBVH::BVH> region_bvhs(n);
    std::vector<ConnectedComponent> components_seal;
    // the regions each sealed component is filled with, in increasing order
    std::vector<std::vector<size_t>> components_regions;
    if (n < needed) {
        logger().warn(
            "tight_seal_geom: only {} tags other than ambient are some tet's only tag, fewer than "
            "min_regions {}, nothing to seal",
            n,
            needed);
    } else if (std::find(region_found.begin(), region_found.end(), false) != region_found.end()) {
        logger().warn(
            "tight_seal_geom: a tag in {} has no tet tagged with it alone, nothing to seal",
            tags);
    } else {
        for (size_t i = 0; i < n; ++i) {
            tight_seal_boundary_bvh(
                tids,
                [&](const size_t tid) { return region[tid] == int(i); },
                region_bvhs[i]);
        }

        // The distance to R, the union of the R_i: from outside R, the distance to its boundary.
        // dR at every vertex, 0 on R's own tets.
        SimpleBVH::BVH bvh_R;
        tight_seal_boundary_bvh(tids, [&](const size_t tid) { return region[tid] >= 0; }, bvh_R);
        const std::function<double(const Vector3d&)> dist_R = [&](const Vector3d& p) {
            return tight_seal::bvh_distance(bvh_R, p);
        };
        std::vector<bool> on_R(vert_capacity(), false);
        for (const size_t tid : tids) {
            if (region[tid] >= 0) {
                for (const size_t v : oriented_tet_vids(tid)) {
                    on_R[v] = true;
                }
            }
        }
        std::vector<double> dR(vert_capacity(), 0.);
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            dR[vid] = on_R[vid] ? 0. : dist_R(pos(vid));
        }

        const double tol = trace_rel_tol * radius;

        // 1. The dilation surface S, the level set d(x, R) = d/2 (the boundary of R + B),
        // traced on the edges that cross it and joined in each tet. The closing is the part
        // of R + B at least d/2 from S, so J ends where d(x, S) falls to d/2.
        SimpleBVH::BVH bvh_S;
        size_t n_S_traced = 0;
        size_t n_S_failed = 0;
        const bool has_S = tight_seal_dilation_surface_bvh(
            tids,
            region,
            dR,
            dist_R,
            radius,
            tol,
            bvh_S,
            n_S_traced,
            n_S_failed);
        logger().info(
            "tight_seal_geom: dilation surface traced on {} edges to d(x, R) = d/2 ({} left "
            "the edge)",
            n_S_traced,
            n_S_failed);
        const std::function<double(const Vector3d&)> dist_S = [&](const Vector3d& p) {
            return has_S ? tight_seal::bvh_distance(bvh_S, p)
                         : std::numeric_limits<double>::infinity();
        };

        // 2. A vertex is inside if it lies in R + B (d(v, R) < d/2) at least d/2 from S: in
        // the closing. It counts as inside down to (1 - tol) x d/2, so one on the level set
        // whatever the rounding is never split off, and every vertex of R is inside: R lies
        // in the closing (each point of S is d/2 from R, so at least d/2 from any vertex of
        // R), and S, joined linearly in each tet, can pass closer to R than the level set by
        // more than the tolerance (in a coarse tet beside a gap's mouth, where the level set
        // bulges into the gap). Every tet outside R with a vertex inside is processed: each
        // of its edges from an inside vertex to an outside one is split where d(x, S) falls
        // to d/2, sphere traced from the inside end: d(x, S) is 1-Lipschitz, so a step of
        // d(x, S) - d/2 cannot cross the level set, and the trace stops within the tolerance
        // of it. An edge whose inside end is within the tolerance of the level set, or below
        // it (a vertex of R), is not split: the trace stops at once. So no edge on R's
        // boundary is split (both ends are inside). J is then the processed tets whose
        // vertices are all inside.
        std::vector<char> inside(vert_capacity(), 0);
        size_t n_inside = 0; // inside, not a vertex of R
        size_t n_band = 0; // of them, within the tolerance of d/2 from S
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            if (on_R[vid]) {
                inside[vid] = 1;
                continue;
            }
            const double ds = dR[vid] < radius ? dist_S(pos(vid)) : 0.;
            inside[vid] = ds >= radius - tol;
            n_inside += inside[vid];
            n_band += inside[vid] && ds <= radius + tol;
        }
        logger().info(
            "tight_seal_geom: {} vertices inside J off R ({} of them within the tolerance of "
            "d/2 from S), and every vertex of R",
            n_inside,
            n_band);
        std::vector<size_t> to_process;
        for (const size_t tid : tids) {
            const auto vs = oriented_tet_vids(tid);
            if (region[tid] < 0 &&
                (inside[vs[0]] || inside[vs[1]] || inside[vs[2]] || inside[vs[3]])) {
                to_process.push_back(tid);
            }
        }
        // the edges J's boundary crosses: from inside to outside
        std::vector<simplex::Edge> split_edges;
        for (const size_t tid : to_process) {
            const auto vs = oriented_tet_vids(tid);
            for (const auto& [a, b] : tight_seal::tet_edges) {
                if (inside[vs[a]] != inside[vs[b]]) {
                    split_edges.emplace_back(vs[a], vs[b]);
                }
            }
        }
        vector_unique(split_edges);

        // the processed tets, kept up to date as their edges are split
        std::unordered_set<size_t> processed(to_process.begin(), to_process.end());
        Vector3d trace_point;
        Vector3d trace_dir;
        // linear along the edge, zero at the traced point, so the split lands on it
        m_voronoi_split_fn = [&](const Vector3d& p) -> double {
            return (p - trace_point).dot(trace_dir);
        };
        size_t n_split = 0;
        size_t n_not_split = 0;
        for (const simplex::Edge& e : split_edges) {
            const auto [e0, e1] = e.vertices();
            const Vector3d a = pos(inside[e0] ? e0 : e1);
            const Vector3d b = pos(inside[e0] ? e1 : e0);
            const double L = (b - a).norm();
            trace_dir = (b - a) / L;
            // No split if the inside end is already on the level set (within the tolerance)
            // or below it (a vertex of R), or the trace leaves the edge.
            const auto t = tight_seal::sphere_trace(dist_S, a, trace_dir, L, radius, tol, false);
            if (!t || !(*t > 0.)) {
                ++n_not_split;
                continue;
            }
            trace_point = a + *t * trace_dir;
            const size_t v_new =
                tight_seal_split_tracked(e, processed, filled, [](const size_t, const size_t) {});
            if (v_new == size_t(-1)) {
                ++n_not_split;
                continue;
            }
            ++n_split;
            if (v_new >= inside.size()) {
                inside.resize(v_new + 1, 0);
            }
            inside[v_new] = 1; // on the level set, J's boundary
        }
        m_voronoi_split_fn = nullptr;

        std::vector<size_t> J_tets;
        std::vector<char> in_J(tet_capacity(), 0);
        for (const size_t tid : processed) {
            const auto vs = oriented_tet_vids(tid);
            if (inside[vs[0]] && inside[vs[1]] && inside[vs[2]] && inside[vs[3]]) {
                J_tets.push_back(tid);
                in_J[tid] = 1;
            }
        }
        std::sort(J_tets.begin(), J_tets.end());
        logger().info(
            "tight_seal_geom: processed {} tets, split {} edges where d(x, S) falls to d/2 "
            "({} not split: the trace started on the level set or below it, or left the edge, "
            "or the split was refused); J holds {} tets",
            to_process.size(),
            n_split,
            n_not_split,
            J_tets.size());


        // The spatial selection keeps the tets of J whose tags satisfy it, every tet by
        // default, before J is split into its components.
        if (spatial_selection) {
            const size_t n_before = J_tets.size();
            std::vector<size_t> kept;
            for (const size_t id : J_tets) {
                if (spatial_selection->eval(m_tet_attribute[id].tags)) {
                    kept.push_back(id);
                } else {
                    in_J[id] = 0;
                }
            }
            J_tets = std::move(kept);
            logger().info(
                "tight_seal_geom: the spatial selection {} keeps {} of the {} tets of J",
                spatial_selection->to_string(),
                J_tets.size(),
                n_before);
        }

        // 3. Connected components of J (through faces); keep the ones that share a face with
        // every R_i (with detected tags: with min_regions R_i or more, which the component is
        // then filled with). Regions are found by tag (region_of): the tets made by the splits
        // above have no entry in `region`.
        std::vector<int> comp_id(tet_capacity(), -1);
        size_t n_components = 0;
        size_t n_too_many = 0; // components bordering more than three regions
        for (const size_t tid : J_tets) {
            if (comp_id[tid] != -1) {
                continue;
            }
            ConnectedComponent comp;
            std::vector<bool> borders(n, false);
            comp.cells.push_back(tid);
            comp_id[tid] = int(n_components);
            for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
                const size_t cur = comp.cells[i];
                comp.volume += tet_volume(cur);
                for (int j = 0; j < 4; ++j) {
                    const auto t_opp = tuple_from_face(cur, j).switch_tetrahedron(*this);
                    if (!t_opp) {
                        comp.touches_boundary = true;
                        continue;
                    }
                    const size_t nbr = t_opp->tid(*this);
                    const int r = region_of(nbr);
                    if (r >= 0) {
                        borders[r] = true;
                    }
                    if (!in_J[nbr] || comp_id[nbr] != -1) {
                        continue;
                    }
                    comp_id[nbr] = int(n_components);
                    comp.cells.push_back(nbr);
                }
            }
            ++n_components;
            std::vector<size_t> comp_regions;
            for (size_t i = 0; i < n; ++i) {
                if (borders[i]) {
                    comp_regions.push_back(i);
                }
            }
            // The Voronoi fill handles two or three regions per component: one bordering more
            // (only detected tags can give that; at most three are listed) is left alone.
            if (!fill_mode && comp_regions.size() > 3) {
                std::vector<std::string> names;
                for (const size_t i : comp_regions) {
                    const auto it = m_tag_id_to_name.find(tags[i]);
                    names.push_back(
                        it != m_tag_id_to_name.end() ? it->second : std::to_string(tags[i]));
                }
                logger().warn(
                    "tight_seal_geom: a component of J ({} tets) borders {} regions {}, more "
                    "than three: detected but not filled",
                    comp.cells.size(),
                    comp_regions.size(),
                    names);
                ++n_too_many;
                continue;
            }
            if (comp_regions.size() >= needed) {
                if (detected) {
                    std::vector<int64_t> comp_tags;
                    for (const size_t i : comp_regions) {
                        comp_tags.push_back(tags[i]);
                    }
                    logger().info(
                        "tight_seal_geom: component {} ({} tets) borders tags {}",
                        components_seal.size(),
                        comp.cells.size(),
                        comp_tags);
                }
                std::sort(comp.cells.begin(), comp.cells.end());
                components_seal.push_back(std::move(comp));
                components_regions.push_back(std::move(comp_regions));
            }
        }
        logger().info(
            "tight_seal_geom: J has {} components; {} of them border {} {}; {} border more "
            "than three and are not filled",
            n_components,
            components_seal.size(),
            detected ? fmt::format(
                           "{}{} of the detected tags",
                           needed,
                           fill_mode ? " or more" : (needed < 3 ? " to 3" : ""))
                     : std::string("all tags"),
            tags,
            n_too_many);

        // J may not touch the mesh's boundary at all, not even at a vertex, in any component: the
        // topology of a region that reaches the boundary is much harder to reason about.
        const size_t n_on_boundary = tight_seal_cells_on_boundary(J_tets);
        if (n_on_boundary > 0) {
            size_t n_to_fill = 0;
            for (const ConnectedComponent& comp : components_seal) {
                n_to_fill += tight_seal_cells_on_boundary(comp.cells);
            }
            log_and_throw_error(
                "tight_seal_geom: the join region touches the mesh boundary: {} of its {} tets "
                "have a vertex on it, {} of them in parts to fill. Keep the regions more than "
                "distance / 2 from the boundary",
                n_on_boundary,
                J_tets.size(),
                n_to_fill);
        }
    }

    // Each component is filled by tight_seal_topology_fill in "fill" mode (no split), otherwise by
    // the Voronoi fill of each component with its regions, by enumeration
    // (MultiVoronoiEnumeration.hpp): each vertex gets the set of the component's regions nearest to
    // it (those it touches, for a vertex on a region), and each tet is split as its corners' sets
    // require (its pierced faces, then its edges) and labelled with the region they share.
    Vector3d lo = Vector3d::Constant(std::numeric_limits<double>::infinity());
    Vector3d hi = -lo;
    for (const Tuple& t : get_vertices()) {
        lo = lo.cwiseMin(pos(t.vid(*this)));
        hi = hi.cwiseMax(pos(t.vid(*this)));
    }
    const double tie_eps = 1e-10 * (hi - lo).norm(); // distances this close count as a tie

    // The component of each tet, kept up to date through the splits. Components are connected
    // through faces, so two of them can share an edge, and splitting it for one also splits the
    // other's tets: each component's tets are read from here when its turn comes.
    std::vector<int> cell_comp(tet_capacity(), -1);
    for (size_t c = 0; c < components_seal.size(); ++c) {
        for (const size_t tid : components_seal[c].cells) {
            cell_comp[tid] = int(c);
        }
    }

    // The fill's view of the mesh, for one component: label l is region regions[l].
    struct FillAdapter
    {
        SimWildMesh& m;
        std::unordered_set<size_t>& cells;
        std::vector<char>& filled;
        std::vector<int>& cell_comp;
        const std::vector<size_t>& regions;
        const std::vector<SimpleBVH::BVH>& bvhs;
        const std::vector<int64_t>& tags;
        multi_voronoi::LabelSet all;
        double eps;

        std::array<size_t, 4> tet_vertices(const size_t t) const { return m.oriented_tet_vids(t); }
        Vector3d position(const size_t v) const { return m.m_vertex_attribute[v].m_posf; }
        double distance(const Vector3d& x, const int l) const
        {
            return tight_seal::bvh_distance(bvhs[regions[l]], x);
        }
        multi_voronoi::LabelSet initial_labels(const size_t v) const
        {
            // the regions of the tets at v; a filled tet (of another component) carries a
            // region's tag too, but is no region
            multi_voronoi::LabelSet s = 0;
            for (const size_t t : m.get_one_ring_tids_for_vertex(v)) {
                const CellTag& tag = m.m_tet_attribute[t].tags;
                if (tag.size() != 1 || filled[t]) {
                    continue;
                }
                for (size_t l = 0; l < regions.size(); ++l) {
                    if (tags[regions[l]] == *tag.begin()) {
                        s |= multi_voronoi::label_bit(int(l));
                    }
                }
            }
            if (s != 0) {
                return s;
            }
            return multi_voronoi::nearest_labels(
                [this](const Vector3d& x, const int l) { return distance(x, l); },
                position(v),
                all,
                eps);
        }
        bool alive(const size_t t) const { return cells.count(t) > 0; }
        size_t split_edge(const size_t v0, const size_t v1, const Vector3d& x)
        {
            // linear along the edge, zero at x, so split_adjust_position puts the vertex there
            const Vector3d dir = (position(v1) - position(v0)).normalized();
            m.m_voronoi_split_fn = [x, dir](const Vector3d& p) { return (p - x).dot(dir); };
            const size_t v = m.tight_seal_split_tracked(
                simplex::Edge(v0, v1),
                cells,
                filled,
                [](const size_t, const size_t) {},
                &cell_comp);
            m.m_voronoi_split_fn = nullptr;
            return v;
        }
        size_t split_face(const size_t v0, const size_t v1, const size_t v2, const Vector3d& x)
        {
            return m.tight_seal_split_face_tracked({{v0, v1, v2}}, x, cells, filled, &cell_comp);
        }
        std::vector<size_t> tets_around(const size_t v) const
        {
            std::vector<size_t> out;
            for (const size_t t : m.get_one_ring_tids_for_vertex(v)) {
                if (cells.count(t) > 0) {
                    out.push_back(t);
                }
            }
            return out;
        }
        // replace the tet's tags of the component's regions with label l's; other tags stay
        void set_label(const size_t t, const int l)
        {
            CellTag& tag = m.m_tet_attribute[t].tags;
            for (const size_t j : regions) {
                tag.erase(tags[j]);
            }
            tag.insert(tags[regions[l]]);
            filled[t] = 1;
        }
    };

    if (split_fill) {
        tight_seal_split_fill(components_seal, components_regions, tags, region_bvhs, filled);
    } else if (fill_mode) {
        tight_seal_topology_fill(components_seal, components_regions, tags, region_bvhs, filled);
    } else {
        multi_voronoi::FillStats3D fill_stats;
        for (size_t c = 0; c < components_seal.size(); ++c) {
            std::vector<size_t> cells;
            for (const Tuple& t : get_tets()) {
                const size_t tid = t.tid(*this);
                if (tid < cell_comp.size() && cell_comp[tid] == int(c)) {
                    cells.push_back(tid);
                }
            }
            const std::vector<size_t>& regions = components_regions[c];
            if (regions.size() > size_t(multi_voronoi::max_labels)) {
                log_and_throw_error(
                    "tight_seal_geom: a component borders {} regions, more than the fill's {}",
                    regions.size(),
                    multi_voronoi::max_labels);
            }
            const multi_voronoi::LabelSet all =
                regions.size() == size_t(multi_voronoi::max_labels)
                    ? ~multi_voronoi::LabelSet(0)
                    : multi_voronoi::label_bit(int(regions.size())) - 1;
            std::unordered_set<size_t> comp_cells(cells.begin(), cells.end());
            FillAdapter adapter{
                *this,
                comp_cells,
                filled,
                cell_comp,
                regions,
                region_bvhs,
                tags,
                all,
                tie_eps};
            fill_stats += multi_voronoi::fill_component_3d(adapter, cells, all, tie_eps);
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
        }
        if (!components_seal.empty()) {
            logger().info(
                "tight_seal_geom: Voronoi fill of {} components: tets planned as {} sheet, {} "
                "slab, "
                "{} Y, {} other; {} edges crossed twice; {} face splits, {} edge splits; {} tets "
                "revisited",
                components_seal.size(),
                fill_stats.sheet,
                fill_stats.slab,
                fill_stats.y,
                fill_stats.other,
                fill_stats.fake_edges,
                fill_stats.face_splits,
                fill_stats.edge_splits,
                fill_stats.visits);
            if (fill_stats.other > 0 || fill_stats.refused_splits > 0 ||
                fill_stats.fallback_labels > 0 || fill_stats.gave_up) {
                logger().warn(
                    "tight_seal_geom: Voronoi fill: {} tets of no known case, {} splits refused, "
                    "{} "
                    "tets labelled by their centroid's nearest region{}",
                    fill_stats.other,
                    fill_stats.refused_splits,
                    fill_stats.fallback_labels,
                    fill_stats.gave_up ? ", the visit cap was reached" : "");
            }
        }
    }

    // Sanity checks, on the output alone rather than on the components above: no tet is
    // inverted; each filled tet carries exactly one of the regions' tags; and each part of the
    // filled tets, connected through faces, borders two regions or more. A part bordering a
    // single region would be a concave feature of that body, filled.
    if (sanity) {
        tight_seal_geom_sanity(tags, filled);
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags)
{
    for (const auto& expressions : intersecting_tags) {
        for (const auto& expr : expressions) {
            if (!expr->contains_only_and()) {
                log_and_throw_error(
                    "Only AND expressions are supported for resolving intersections. Expression "
                    "is {}",
                    expr->to_string());
            }
        }

        const ExprPtr& expr1 = expressions[0];
        const ExprPtr& expr2 = expressions[1];
        auto intersection_region = std::make_shared<expression_parser::AndExpr>(expr1, expr2);

        std::vector<ConnectedComponent> components =
            compute_connected_components(intersection_region);
        if (components.empty()) {
            logger().info("No cell contains the expression {}", intersection_region->to_string());
            continue;
        }
        logger().info(
            "Resolve {} intersections in region {}",
            components.size(),
            intersection_region->to_string());

        const CellTag tags1 = expr1->tags_involved();
        const CellTag tags2 = expr2->tags_involved();
        const CellTag intersection_tags = intersection_region->tags_involved();

        // remove tags from components
        for (const ConnectedComponent& comp : components) {
            for (const size_t tid : comp.cells) {
                auto& tag = m_tet_attribute[tid].tags;
                for (const size_t tt : intersection_tags) {
                    tag.erase(tt);
                }
            }
        }

        std::vector<CellTag> tag_vec(2);
        tag_vec[0] = tags1;
        tag_vec[1] = tags2;
        seal_connected_components(tag_vec, components);
    }

    // report other intersections
    std::map<size_t, std::set<size_t>> intersections;
    for (const Tuple& t : get_tets()) {
        const auto& tags = m_tet_attribute[t.tid(*this)].tags;
        if (tags.size() < 2) {
            continue;
        }
        for (const size_t tag : tags) {
            auto& ins = intersections[tag];
            ins.insert(tags.begin(), tags.end());
        }
    }
    if (intersections.size() != 0) {
        logger().info("Remaining intersections:");
        for (const auto& [tag, others] : intersections) {
            logger().info("Tag {} intersections: {}", tag, others);
        }
    } else {
        logger().info("Tags are intersection free.");
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::replace_tags(
    const std::vector<CellTag>& tags_in,
    const std::vector<CellTag>& tags_out)
{
    if (tags_in.size() != tags_out.size()) {
        log_and_throw_error(
            "replace_tags: tags_in and tags_out must have the same size. Got {} and {}.",
            tags_in.size(),
            tags_out.size());
    }

    for (const Tuple& t : get_tets()) {
        CellTag& tags = m_tet_attribute[t.tid(*this)].tags;
        std::vector<bool> found_tags(tags_in.size(), false);
        for (size_t i = 0; i < tags_in.size(); ++i) {
            if (set_includes(tags, tags_in[i])) {
                found_tags[i] = true;
            }
        }
        // remove "tags_in" from tags
        for (size_t i = 0; i < tags_in.size(); ++i) {
            if (found_tags[i]) {
                for (const int64_t tt : tags_in[i]) {
                    tags.erase(tt);
                }
            }
        }
        // add "tags_out" to tags
        for (size_t i = 0; i < tags_out.size(); ++i) {
            if (found_tags[i]) {
                tags.insert(tags_out[i].begin(), tags_out[i].end());
            }
        }
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMesh::tag_priority(const std::vector<int64_t>& tags_order)
{
    for (const Tuple& t : get_tets()) {
        CellTag& tags = m_tet_attribute[t.tid(*this)].tags;
        for (size_t i = 0; i < tags_order.size(); ++i) {
            if (tags.find(tags_order[i]) != tags.end()) {
                // found the highest priority tag, remove all lower priority tags
                for (size_t j = i + 1; j < tags_order.size(); ++j) {
                    tags.erase(tags_order[j]);
                }
                break;
            }
        }
    }

    m_F_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

} // namespace wmtk::components::simwild
