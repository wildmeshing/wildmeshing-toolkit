#include "SimWildMeshTri.hpp"
#include "TightSealGeom.hpp"

namespace wmtk::components::simwild::tri {

std::vector<ConnectedComponent> SimWildMeshTri::compute_connected_components(
    const CellTag& tag_in) const
{
    std::vector<int> comp_id(tri_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (comp_id[fid] != -1) {
            continue;
        }

        const CellTag& tag = m_face_attribute[fid].tags;

        // check if face contains the tag_in tags
        if (!set_includes(tag, tag_in)) {
            continue;
        }

        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();

        comp.cells.push_back(fid);
        comp_id[fid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += triangle_area(cur);
            for (int j = 0; j < 3; ++j) {
                const Tuple edge_tup = tuple_from_edge(cur, j);
                const auto t_opp = edge_tup.switch_face(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->fid(*this);
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
                const CellTag& ntag = m_face_attribute[nbr].tags;
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

std::vector<ConnectedComponent> SimWildMeshTri::compute_connected_components(
    const ExprPtr& expr) const
{
    std::vector<int> comp_id(tri_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (comp_id[fid] != -1) {
            continue;
        }

        const CellTag& tag = m_face_attribute[fid].tags;

        // check if face contains the tag_in tags
        if (!expr->eval(tag)) {
            continue;
        }

        const int comp_idx = (int)components.size();
        ConnectedComponent& comp = components.emplace_back();

        comp.cells.push_back(fid);
        comp_id[fid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += triangle_area(cur);
            for (int j = 0; j < 3; ++j) {
                const Tuple edge_tup = tuple_from_edge(cur, j);
                const auto t_opp = edge_tup.switch_face(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->fid(*this);
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
                const CellTag& ntag = m_face_attribute[nbr].tags;
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

std::vector<ConnectedComponent> SimWildMeshTri::find_holes(const std::vector<CellTag>& tag_in) const
{
    std::vector<int> comp_id(tri_capacity(), -1);
    std::vector<ConnectedComponent> components;

    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (comp_id[fid] != -1) {
            continue;
        }

        const CellTag& tag = m_face_attribute[fid].tags;

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

        comp.cells.push_back(fid);
        comp_id[fid] = comp_idx;

        for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
            const size_t cur = comp.cells[i];
            comp.volume += triangle_area(cur);
            for (int j = 0; j < 3; ++j) {
                const Tuple edge_tup = tuple_from_edge(cur, j);
                const auto t_opp = edge_tup.switch_face(*this);
                if (!t_opp) {
                    comp.touches_boundary = true;
                    continue;
                }
                const size_t nbr = t_opp->fid(*this);
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
                const CellTag& ntag = m_face_attribute[nbr].tags;
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

void SimWildMeshTri::compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& E) const
{
    // extract boundary edges
    std::vector<Vector2i> edges;
    std::vector<Vector2d> vertices;
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (!set_includes(m_face_attribute[fid].tags, tag)) {
            continue;
        }

        for (int j = 0; j < 3; ++j) {
            const Tuple edge_tup = tuple_from_edge(fid, j);
            const auto t_opp = edge_tup.switch_face(*this);
            if (!t_opp) {
                continue; // boundary
            }
            const size_t nbr = t_opp->fid(*this);
            const CellTag& ntag = m_face_attribute[nbr].tags;
            if (!set_includes(ntag, tag)) {
                // found a component boundary, add to BVH
                size_t v0 = edge_tup.vid(*this);
                size_t v1 = edge_tup.switch_vertex(*this).vid(*this);
                size_t nv = vertices.size();
                edges.emplace_back(nv, nv + 1);
                vertices.push_back(m_vertex_attribute.at(v0).m_posf);
                vertices.push_back(m_vertex_attribute.at(v1).m_posf);
            }
        }
    }

    assert(2 * edges.size() == vertices.size());

    V.resize(vertices.size(), 2);
    for (size_t i = 0; i < vertices.size(); ++i) {
        V.row(i) = vertices[i];
    }
    E.resize(edges.size(), 2);
    for (size_t i = 0; i < edges.size(); ++i) {
        E.row(i) = edges[i];
    }
}

void SimWildMeshTri::fill_holes_topo(const std::vector<CellTag>& fill_holes_tags, double threshold)
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
            for (const size_t fid : comp.cells) {
                auto& tag = m_face_attribute[fid].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::seal_connected_components(
    const std::vector<CellTag>& tag_set,
    const std::vector<ConnectedComponent>& components)
{
    auto get_center = [&](const size_t fid) {
        const auto vs = oriented_tri_vids(fid);
        const Vector2d& p0 = m_vertex_attribute[vs[0]].m_posf;
        const Vector2d& p1 = m_vertex_attribute[vs[1]].m_posf;
        const Vector2d& p2 = m_vertex_attribute[vs[2]].m_posf;
        return (p0 + p1 + p2) / 3;
    };

    // BVH for each tag_set
    std::vector<SimpleBVH::BVH> bvhs;
    for (const auto& tag : tag_set) {
        MatrixXd V;
        MatrixXi E;
        compute_tag_boundary(tag, V, E);
        auto& bvh = bvhs.emplace_back();
        bvh.init(V, E, 0);
    }

    // build SDF
    m_voronoi_split_fn = [&](const Vector2d& p) -> double {
        double sq_dist;
        Vector2d nearest_point;

        bvhs[0].nearest_facet(p, nearest_point, sq_dist);
        const double da = std::sqrt(sq_dist);

        bvhs[1].nearest_facet(p, nearest_point, sq_dist);
        const double db = std::sqrt(sq_dist);

        return da - db;
    };
    struct ClearVoronoiSplit
    {
        std::function<double(const Vector2d&)>& fn;
        ~ClearVoronoiSplit() { fn = nullptr; }
    } clear_voronoi_split{m_voronoi_split_fn};

    // seal holes
    for (const ConnectedComponent& hole : components) {
        std::vector<simplex::Edge> split_edges;
        std::unordered_set<size_t> vids;
        std::unordered_set<size_t> vids_left;
        for (const size_t fid : hole.cells) {
            const Vector2d p = get_center(fid);
            const double d = m_voronoi_split_fn(p);
            auto& tag = m_face_attribute[fid].tags;
            if (d < 0) {
                tag.insert(tag_set[0].begin(), tag_set[0].end());
            } else {
                tag.insert(tag_set[1].begin(), tag_set[1].end());
            }

            for (int j = 0; j < 3; ++j) {
                const Tuple t = tuple_from_edge(fid, j);
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

        std::vector<Tuple> new_tris;
        std::unordered_set<size_t> new_vertices;
        for (const simplex::Edge& e : split_edges) {
            const auto [t, eid] = tuple_from_edge(e.vertices());
            reserve_free_slots(cell_slot_bound(t), 1); // serial: grow rather than refuse
            if (!split_edge(t, new_tris)) {
                continue;
            }

            const size_t v_new = m_last_split_vertex;
            std::vector<size_t> fids = get_one_ring_fids_for_vertex(v_new);

            new_vertices.insert(v_new);

            for (const size_t fid : fids) {
                const auto vs = oriented_tri_vids(fid);
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
                        // check the other vertex if it is a hole vertex
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

                // const Vector2d& p = m_vertex_attribute[vid].m_posf;
                // const double d = m_voronoi_split_fn(p);
                // auto& tag = m_face_attribute[fid].tags;
                // if (d < 0) {
                //     for (const size_t tt : tag_set[1]) {
                //         tag.erase(tt);
                //     }
                //     tag.insert(tag_set[0].begin(), tag_set[0].end());
                // } else {
                //     for (const size_t tt : tag_set[0]) {
                //         tag.erase(tt);
                //     }
                //     tag.insert(tag_set[1].begin(), tag_set[1].end());
                // }
                auto& tag = m_face_attribute[fid].tags;
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
        }
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}

void SimWildMeshTri::keep_largest_connected_component(
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
            for (const size_t fid : components[i].cells) {
                auto& tag = m_face_attribute[fid].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::tight_seal_topo(
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

            for (const size_t fid : hole.cells) {
                for (int j = 0; j < 3; ++j) {
                    const Tuple edge_tup = tuple_from_edge(fid, j);
                    const auto t_opp = edge_tup.switch_face(*this);
                    if (!t_opp) {
                        // boundary
                        continue;
                    }
                    const size_t nbr = t_opp->fid(*this);
                    const CellTag& ntag = m_face_attribute[nbr].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::tight_seal_geom(
    const std::vector<int64_t>& listed_tags,
    const double distance,
    const double trace_rel_tol,
    const std::string& selection,
    const int64_t min_regions,
    const bool sanity)
{
    const bool detected = selection == "detected";
    if (!detected && selection != "explicit") {
        log_and_throw_error(
            "tight_seal_geom: selection must be \"explicit\" or \"detected\". Input was {}",
            selection);
    }
    if (detected && min_regions < 2) {
        log_and_throw_error(
            "tight_seal_geom: min_regions must be at least 2. Input was {}",
            min_regions);
    }
    if (!detected && min_regions != 2) {
        logger().warn(
            "tight_seal_geom: min_regions {} is ignored with selection \"explicit\", which fills "
            "the parts that border every listed tag",
            min_regions);
    }
    // the tags of the regions: the listed ones, or every tag but ambient that some face has alone
    std::vector<int64_t> tags;
    if (detected) {
        if (!listed_tags.empty()) {
            logger().warn(
                "tight_seal_geom: selection \"detected\" ignores the listed tags {}",
                listed_tags);
        }
        CellTag found;
        for (const Tuple& t : get_faces()) {
            const CellTag& tag = m_face_attribute[t.fid(*this)].tags;
            if (tag.size() == 1 && !tight_seal::is_ambient(m_tag_id_to_name, *tag.begin())) {
                found.insert(*tag.begin());
            }
        }
        tags.assign(found.begin(), found.end());
    } else {
        tags = listed_tags;
        if (tags.size() < 2) {
            log_and_throw_error("tight_seal_geom needs at least two tags. Input was {}", tags);
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

    const auto pos = [&](const size_t vid) -> const Vector2d& {
        return m_vertex_attribute[vid].m_posf;
    };

    // region[fid] = i for the faces of R_i (tagged tags[i] and nothing else), -1 for E
    std::vector<size_t> fids;
    std::vector<int> region(tri_capacity(), -1);
    std::vector<bool> region_found(n, false);
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        fids.push_back(fid);
        const CellTag& tag = m_face_attribute[fid].tags;
        if (tag.size() != 1) {
            continue;
        }
        const auto it = std::find(tags.begin(), tags.end(), *tag.begin());
        if (it != tags.end()) {
            region[fid] = int(it - tags.begin());
            region_found[region[fid]] = true;
        }
    }

    // m_voronoi_split_fn places the vertex of each split below (split_adjust_position)
    tight_seal::ClearOnExit<std::function<double(const Vector2d&)>> clear_voronoi_split{
        m_voronoi_split_fn};

    // filled[fid]: the Voronoi fill set the face's region. A split's children inherit it from
    // their parent, so it stays right whichever component's split cuts the face. For the sanity
    // check.
    std::vector<char> filled(tri_capacity(), 0);

    std::vector<SimpleBVH::BVH> region_bvhs(n);
    std::vector<ConnectedComponent> components_seal;
    // the regions each sealed component is filled with, in increasing order
    std::vector<std::vector<size_t>> components_regions;
    if (n < needed) {
        logger().warn(
            "tight_seal_geom: only {} tags other than ambient are some face's only tag, fewer "
            "than min_regions {}, nothing to seal",
            n,
            needed);
    } else if (std::find(region_found.begin(), region_found.end(), false) != region_found.end()) {
        logger().warn(
            "tight_seal_geom: a tag in {} has no face tagged with it alone, nothing to seal",
            tags);
    } else {
        for (size_t i = 0; i < n; ++i) {
            tight_seal_boundary_bvh(
                fids,
                [&](const size_t fid) { return region[fid] == int(i); },
                region_bvhs[i]);
        }

        // The distance to R, the union of the R_i: from outside R, the distance to its boundary.
        // dR at every vertex, 0 on R's own faces.
        SimpleBVH::BVH bvh_R;
        tight_seal_boundary_bvh(fids, [&](const size_t fid) { return region[fid] >= 0; }, bvh_R);
        const std::function<double(const Vector2d&)> dist_R = [&](const Vector2d& p) {
            return tight_seal::bvh_distance(bvh_R, p);
        };
        std::vector<bool> on_R(vert_capacity(), false);
        for (const size_t fid : fids) {
            if (region[fid] >= 0) {
                for (const size_t v : oriented_tri_vids(fid)) {
                    on_R[v] = true;
                }
            }
        }
        std::vector<double> dR(vert_capacity(), 0.);
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            dR[vid] = on_R[vid] ? 0. : dist_R(pos(vid));
        }
        const auto corners = [&](const std::array<size_t, 3>& vs) {
            return std::array<Vector2d, 3>{{pos(vs[0]), pos(vs[1]), pos(vs[2])}};
        };

        // 1. The faces surely in J, the closing of R by the disk B of radius d/2 minus R (the
        // spec's v2). U^ = R and the faces whose points are all within d/2 of R: inside R + B.
        // V^ = the faces of U^ whose points are all farther than d/2 from dU^, the boundary of U^
        // (outside the mesh counts as outside U^): inside U^ - B, so inside the closing. J^ = V^
        // minus R. Both tests are certified by tight_seal::certify_on_cell, so rounding can only
        // leave a face out.
        std::vector<bool> in_Uh(tri_capacity(), false);
        for (const size_t fid : fids) {
            const auto vs = oriented_tri_vids(fid);
            in_Uh[fid] = region[fid] >= 0 || tight_seal::certify_on_cell(
                                                 corners(vs),
                                                 {{dR[vs[0]], dR[vs[1]], dR[vs[2]]}},
                                                 dist_R,
                                                 radius,
                                                 true,
                                                 tight_seal::certify_depth_2d);
        }
        SimpleBVH::BVH bvh_dUh;
        const bool has_dUh = tight_seal_boundary_bvh(
            fids,
            [&](const size_t fid) { return bool(in_Uh[fid]); },
            bvh_dUh);
        const std::function<double(const Vector2d&)> dist_dUh = [&](const Vector2d& p) {
            return has_dUh ? tight_seal::bvh_distance(bvh_dUh, p)
                           : std::numeric_limits<double>::infinity();
        };
        std::vector<double> dUh(vert_capacity(), -1.); // at the vertices of U^'s faces outside R
        std::vector<bool> in_Jh(tri_capacity(), false);
        size_t n_Uh = 0;
        size_t n_Jh = 0;
        for (const size_t fid : fids) {
            if (!in_Uh[fid] || region[fid] >= 0) {
                continue;
            }
            ++n_Uh;
            const auto vs = oriented_tri_vids(fid);
            for (const size_t v : vs) {
                if (dUh[v] < 0) {
                    dUh[v] = dist_dUh(pos(v));
                }
            }
            in_Jh[fid] = tight_seal::certify_on_cell(
                corners(vs),
                {{dUh[vs[0]], dUh[vs[1]], dUh[vs[2]]}},
                dist_dUh,
                radius,
                false,
                tight_seal::certify_depth_2d);
            n_Jh += in_Jh[fid];
        }

        logger().info(
            "tight_seal_geom: U^ holds {} faces outside R, J^ {} of them, distance {}",
            n_Uh,
            n_Jh,
            distance);

        if (n_Jh > 0) {
            const double tol = trace_rel_tol * radius;

            // 2. The dilation surface S, the level set d(x, R) = d/2 (the boundary of R + B),
            // traced on the edges that cross it and joined in each face. The closing is the part
            // of R + B at least d/2 from S, so J ends where d(x, S) falls to d/2.
            SimpleBVH::BVH bvh_S;
            size_t n_S_traced = 0;
            size_t n_S_failed = 0;
            const bool has_S = tight_seal_dilation_surface_bvh(
                fids,
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
            const std::function<double(const Vector2d&)> dist_S = [&](const Vector2d& p) {
                return has_S ? tight_seal::bvh_distance(bvh_S, p)
                             : std::numeric_limits<double>::infinity();
            };

            // 3. Conservative growth of all of J^. A vertex is inside if it is a vertex of a J^
            // face, or if it lies in R + B (d(v, R) < d/2) with d(v, S) > d/2: in the closing.
            // From J^, the faces outside R are flooded through edges with a vertex inside. Each
            // edge of the flooded faces from an inside vertex to an outside one is split where
            // d(x, S) falls to d/2, sphere traced from the inside end: d(x, S) is 1-Lipschitz, so
            // a step of d(x, S) - d/2 cannot cross the level set, and the trace stops within the
            // tolerance of it (as TopoOffsetTriMesh::edge_split_sphere_trace). Edges on R's
            // boundary are split too, and so are the faces of R on them (their children keep
            // R's tags). J is then the flooded faces whose vertices are all inside or on the
            // level set.
            std::vector<char> inside(vert_capacity(), 0);
            for (const Tuple& t : get_vertices()) {
                const size_t vid = t.vid(*this);
                inside[vid] = dR[vid] < radius && dist_S(pos(vid)) > radius;
            }
            std::vector<size_t> flooded;
            std::vector<char> in_flood(tri_capacity(), 0);
            for (const size_t fid : fids) {
                if (!in_Jh[fid]) {
                    continue;
                }
                flooded.push_back(fid);
                in_flood[fid] = 1;
                for (const size_t v : oriented_tri_vids(fid)) {
                    inside[v] = 1;
                }
            }
            for (size_t i = 0; i < flooded.size(); ++i) { // BFS loop with flooded growing
                const size_t cur = flooded[i];
                for (int j = 0; j < 3; ++j) {
                    const Tuple edge_tup = tuple_from_edge(cur, j);
                    if (!inside[edge_tup.vid(*this)] &&
                        !inside[edge_tup.switch_vertex(*this).vid(*this)]) {
                        continue;
                    }
                    const auto t_opp = edge_tup.switch_face(*this);
                    if (!t_opp) {
                        continue;
                    }
                    const size_t nbr = t_opp->fid(*this);
                    if (region[nbr] >= 0 || in_flood[nbr]) {
                        continue;
                    }
                    in_flood[nbr] = 1;
                    flooded.push_back(nbr);
                }
            }
            std::vector<simplex::Edge> split_edges;
            for (const size_t fid : flooded) {
                const auto vs = oriented_tri_vids(fid);
                for (int j = 0; j < 3; ++j) {
                    if (inside[vs[j]] != inside[vs[(j + 1) % 3]]) {
                        split_edges.emplace_back(vs[j], vs[(j + 1) % 3]);
                    }
                }
            }
            vector_unique(split_edges);

            std::unordered_set<size_t> grown(flooded.begin(), flooded.end());
            Vector2d trace_point;
            Vector2d trace_dir;
            // linear along the edge, zero at the traced point, so the split lands on it
            m_voronoi_split_fn = [&](const Vector2d& p) -> double {
                return (p - trace_point).dot(trace_dir);
            };
            size_t n_split = 0;
            size_t n_not_split = 0;
            for (const simplex::Edge& e : split_edges) {
                const auto [e0, e1] = e.vertices();
                const Vector2d a = pos(inside[e0] ? e0 : e1);
                const Vector2d b = pos(inside[e0] ? e1 : e0);
                const double L = (b - a).norm();
                trace_dir = (b - a) / L;
                // No split if the inside end is already on the level set (within the tolerance)
                // or the trace leaves the edge.
                const auto t = tight_seal::sphere_trace(dist_S, a, trace_dir, L, radius, tol, false);
                if (!t || !(*t > 0.)) {
                    ++n_not_split;
                    continue;
                }
                trace_point = a + *t * trace_dir;
                const size_t v_new =
                    tight_seal_split_tracked(e, grown, filled, [](const size_t, const size_t) {});
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

            std::vector<size_t> J_faces;
            std::vector<char> in_J(tri_capacity(), 0);
            for (const size_t fid : grown) {
                const auto vs = oriented_tri_vids(fid);
                if (inside[vs[0]] && inside[vs[1]] && inside[vs[2]]) {
                    J_faces.push_back(fid);
                    in_J[fid] = 1;
                }
            }
            std::sort(J_faces.begin(), J_faces.end());
            logger().info(
                "tight_seal_geom: growth flooded {} faces, split {} edges where d(x, S) falls to "
                "d/2 ({} not split: the trace started on the level set or left the edge, or the "
                "split was refused); J holds {} faces",
                flooded.size(),
                n_split,
                n_not_split,
                J_faces.size());

            // 4. Connected components of J (through edges); keep the ones that share an edge with
            // every R_i (with detected tags: with min_regions R_i or more, which the component is
            // then filled with). Regions are found by tag: the faces of R split above have no
            // entry in `region`.
            const auto region_of = [&](const size_t fid) {
                const CellTag& tag = m_face_attribute[fid].tags;
                if (tag.size() != 1) {
                    return -1;
                }
                const auto it = std::find(tags.begin(), tags.end(), *tag.begin());
                return it == tags.end() ? -1 : int(it - tags.begin());
            };
            std::vector<int> comp_id(tri_capacity(), -1);
            size_t n_components = 0;
            for (const size_t fid : J_faces) {
                if (comp_id[fid] != -1) {
                    continue;
                }
                ConnectedComponent comp;
                std::vector<bool> borders(n, false);
                comp.cells.push_back(fid);
                comp_id[fid] = int(n_components);
                for (size_t i = 0; i < comp.cells.size(); ++i) { // BFS loop with comp.cells growing
                    const size_t cur = comp.cells[i];
                    comp.volume += triangle_area(cur);
                    for (int j = 0; j < 3; ++j) {
                        const auto t_opp = tuple_from_edge(cur, j).switch_face(*this);
                        if (!t_opp) {
                            comp.touches_boundary = true;
                            continue;
                        }
                        const size_t nbr = t_opp->fid(*this);
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
                if (comp_regions.size() >= needed) {
                    if (detected) {
                        std::vector<int64_t> comp_tags;
                        for (const size_t i : comp_regions) {
                            comp_tags.push_back(tags[i]);
                        }
                        logger().info(
                            "tight_seal_geom: component {} ({} faces) borders tags {}",
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
                "tight_seal_geom: J has {} components; {} of them border {} {}",
                n_components,
                components_seal.size(),
                detected ? fmt::format("{} or more of the detected tags", needed)
                         : std::string("all tags"),
                tags);
        }
    }

    // Voronoi fill, of each component with its regions. A point's region is the one of them
    // nearest to it. An edge whose endpoints are in regions a and b is split where d_a = d_b, by
    // split_adjust_position's bisection.
    size_t split_a = 0;
    size_t split_b = 0;
    m_voronoi_split_fn = [&](const Vector2d& p) -> double {
        return tight_seal::bvh_distance(region_bvhs[split_a], p) -
               tight_seal::bvh_distance(region_bvhs[split_b], p);
    };

    size_t n_splits = 0;
    for (size_t c = 0; c < components_seal.size(); ++c) {
        const ConnectedComponent& comp = components_seal[c];
        const std::vector<size_t>& regions = components_regions[c];
        // replace the face's tags of the component's regions with tags[i]; other tags stay
        const auto set_region = [&](const size_t fid, const size_t i) {
            CellTag& tag = m_face_attribute[fid].tags;
            for (const size_t j : regions) {
                tag.erase(tags[j]);
            }
            tag.insert(tags[i]);
            filled[fid] = 1;
        };

        // the component's faces, kept up to date as its edges are split
        std::unordered_set<size_t> comp_faces(comp.cells.begin(), comp.cells.end());
        std::unordered_map<size_t, size_t> vertex_region;
        for (const size_t fid : comp.cells) {
            const auto vs = oriented_tri_vids(fid);
            set_region(
                fid,
                tight_seal::nearest_region(
                    region_bvhs,
                    Vector2d((pos(vs[0]) + pos(vs[1]) + pos(vs[2])) / 3),
                    regions));
            for (const size_t v : vs) {
                if (vertex_region.count(v) == 0) {
                    vertex_region[v] = tight_seal::nearest_region(region_bvhs, pos(v), regions);
                }
            }
        }

        std::vector<simplex::Edge> split_edges;
        for (const size_t fid : comp.cells) {
            const auto vs = oriented_tri_vids(fid);
            for (int j = 0; j < 3; ++j) {
                const size_t v0 = vs[j];
                const size_t v1 = vs[(j + 1) % 3];
                split_a = vertex_region.at(v0);
                split_b = vertex_region.at(v1);
                if (split_a == split_b) {
                    continue;
                }
                // only split edges if their endpoints aren't already on the bisector
                if (m_voronoi_split_fn(pos(v0)) < -1e-20 && m_voronoi_split_fn(pos(v1)) > 1e-20) {
                    split_edges.emplace_back(v0, v1);
                }
            }
        }
        vector_unique(split_edges);

        for (const simplex::Edge& e : split_edges) {
            split_a = vertex_region.at(e.vertices()[0]);
            split_b = vertex_region.at(e.vertices()[1]);
            const size_t v_new = tight_seal_split_tracked(
                e,
                comp_faces,
                filled,
                [&](const size_t fid, const size_t end_vid) {
                    set_region(fid, vertex_region.at(end_vid));
                });
            if (v_new != size_t(-1)) {
                ++n_splits;
            }
        }
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
    if (!components_seal.empty()) {
        logger().info(
            "tight_seal_geom: sealed {} components, {} edges split",
            components_seal.size(),
            n_splits);
    }

    // Sanity checks, on the output alone rather than on the components above: no face is
    // inverted; each filled face carries exactly one of the regions' tags; and each part of the
    // filled faces, connected through edges, borders two regions or more. A part bordering a
    // single region would be a concave feature of that body, filled.
    if (sanity) {
        tight_seal_geom_sanity(tags, filled);
    }

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags)
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
            for (const size_t fid : comp.cells) {
                auto& tag = m_face_attribute[fid].tags;
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
    std::map<size_t, CellTag> intersections;
    for (const Tuple& t : get_faces()) {
        const auto& tags = m_face_attribute[t.fid(*this)].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::replace_tags(
    const std::vector<CellTag>& tags_in,
    const std::vector<CellTag>& tags_out)
{
    if (tags_in.size() != tags_out.size()) {
        log_and_throw_error(
            "replace_tags: tags_in and tags_out must have the same size. Got {} and {}.",
            tags_in.size(),
            tags_out.size());
    }

    for (const Tuple& t : get_faces()) {
        CellTag& tags = m_face_attribute[t.fid(*this)].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

void SimWildMeshTri::tag_priority(const std::vector<int64_t>& tags_order)
{
    for (const Tuple& t : get_faces()) {
        CellTag& tags = m_face_attribute[t.fid(*this)].tags;
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

    m_E_envelope.clear();
    m_V_envelope.clear();
    m_envelope.reset();
}

} // namespace wmtk::components::simwild::tri
