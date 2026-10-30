#include "SimWildMesh.h"

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
    const bool growth,
    const double trace_rel_tol,
    const std::string& selection,
    const int64_t min_regions)
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
    const auto is_ambient = [&](const int64_t tag) {
        const auto it = m_tag_id_to_name.find(tag);
        return it != m_tag_id_to_name.end() && it->second == "ambient";
    };
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
            if (tag.size() == 1 && !is_ambient(*tag.begin())) {
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
            if (is_ambient(tag)) {
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
    if (growth && !(trace_rel_tol > 0 && trace_rel_tol < 1)) {
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
    // the four faces of a tet, as its vertices with one left out
    const auto tet_faces = [](const std::array<size_t, 4>& vs) {
        return std::array<std::array<size_t, 3>, 4>{
            {{{vs[1], vs[2], vs[3]}},
             {{vs[0], vs[2], vs[3]}},
             {{vs[0], vs[1], vs[3]}},
             {{vs[0], vs[1], vs[2]}}}};
    };
    constexpr std::array<std::array<int, 2>, 6> tet_edges = {
        {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};

    // region[tid] = i for the tets of R_i (tagged tags[i] and nothing else), -1 for E
    std::vector<size_t> tids;
    std::vector<int> region(tet_capacity(), -1);
    std::vector<bool> region_found(n, false);
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        tids.push_back(tid);
        const CellTag& tag = m_tet_attribute[tid].tags;
        if (tag.size() != 1) {
            continue;
        }
        const auto it = std::find(tags.begin(), tags.end(), *tag.begin());
        if (it != tags.end()) {
            region[tid] = int(it - tags.begin());
            region_found[region[tid]] = true;
        }
    }

    // BVH of triangles, given as consecutive triples of points. Returns false if there are none.
    const auto triangles_bvh = [](const std::vector<Vector3d>& vertices, SimpleBVH::BVH& bvh) {
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
    };
    // BVH of the boundary of a set of tets: its interfaces and its faces on the mesh boundary.
    // From a point outside the set, the distance to it is the distance to this boundary.
    const auto boundary_bvh = [&](const auto& in_set, SimpleBVH::BVH& bvh) {
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
                vertices.push_back(pos(face_tup.vid(*this)));
                vertices.push_back(pos(face_tup.switch_vertex(*this).vid(*this)));
                vertices.push_back(
                    pos(face_tup.switch_edge(*this).switch_vertex(*this).vid(*this)));
            }
        }
        return triangles_bvh(vertices, bvh);
    };
    const auto bvh_distance = [](const SimpleBVH::BVH& bvh, const Vector3d& p) {
        double sq_dist;
        Vector3d nearest_point;
        bvh.nearest_facet(p, nearest_point, sq_dist);
        return std::sqrt(sq_dist);
    };

    // m_voronoi_split_fn places the vertex of each split below (split_adjust_position)
    struct ClearVoronoiSplit
    {
        std::function<double(const Vector3d&)>& fn;
        ~ClearVoronoiSplit() { fn = nullptr; }
    } clear_voronoi_split{m_voronoi_split_fn};

    // Split edge e, keeping `cells` up to date: the children of a tet in it replace it. Calls
    // on_child(tid, end_vid) for each child in it, end_vid being the endpoint of e the child
    // keeps. Returns the new vertex, or -1 if the split was refused.
    const auto split_tracked = [&](const simplex::Edge& e,
                                   std::unordered_set<size_t>& cells,
                                   const auto& on_child) -> size_t {
        const auto [e0, e1] = e.vertices();
        const Tuple t = tuple_from_edge(e.vertices());

        // Each tet on the edge gives two children, which keep the tet's edge opposite the split
        // edge. Record by that edge whether the tet is in the set.
        const std::vector<size_t> parents = get_incident_tids_for_edge(t);
        std::map<simplex::Edge, bool> parent_in_set;
        for (const size_t tid : parents) {
            parent_in_set[simplex_from_tet(tid).opposite_edge(e)] = cells.count(tid) > 0;
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
            const auto it = parent_in_set.find(simplex::Edge(opp[0], opp[1]));
            if (it == parent_in_set.end()) {
                log_and_throw_error("Could not find the parent of a tet after a split.");
            }
            if (!it->second) {
                continue; // outside the set, keeps its tags
            }
            cells.insert(tid);
            on_child(tid, end_vid);
        }
        return v_new;
    };

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
            boundary_bvh([&](const size_t tid) { return region[tid] == int(i); }, region_bvhs[i]);
        }

        // U: vertices at distance >= d/2 from R, the union of the R_i. The vertices of R's own
        // tets are at distance 0.
        SimpleBVH::BVH bvh_R;
        boundary_bvh([&](const size_t tid) { return region[tid] >= 0; }, bvh_R);
        std::vector<bool> on_R(vert_capacity(), false);
        for (const size_t tid : tids) {
            if (region[tid] >= 0) {
                for (const size_t v : oriented_tet_vids(tid)) {
                    on_R[v] = true;
                }
            }
        }
        std::vector<bool> in_U(vert_capacity(), false);
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            in_U[vid] = !on_R[vid] && bvh_distance(bvh_R, pos(vid)) >= radius;
        }

        // U_mesh: the tets of E whose four vertices are in U
        std::vector<bool> in_U_mesh(tet_capacity(), false);
        std::vector<bool> on_U_mesh(vert_capacity(), false);
        for (const size_t tid : tids) {
            const auto vs = oriented_tet_vids(tid);
            if (region[tid] >= 0 || !in_U[vs[0]] || !in_U[vs[1]] || !in_U[vs[2]] || !in_U[vs[3]]) {
                continue;
            }
            in_U_mesh[tid] = true;
            for (const size_t v : vs) {
                on_U_mesh[v] = true;
            }
        }

        // With growth, U_mesh is cut along the level set d(x, R) = d/2 rather than made of whole
        // tets. Each edge of E from a vertex outside U to one in U is sphere traced from its
        // outer end, outwards from R, to where the distance to R reaches d/2 (within the
        // tolerance, on R's side), and in each tet of E, U_mesh is the part on U's side of the
        // traced points: bounded by the part of each face on U's side and by the cut polygon
        // between the traced points. Nothing is split: only the distance to U_mesh changes.
        // Whole tets fall short of U by up to a tet, which left a sheet of J along every body's
        // outer side; this U_mesh holds U up to the tolerance. As in 2D.
        SimpleBVH::BVH bvh_U;
        bool has_U_mesh = false;
        if (!growth) {
            has_U_mesh =
                boundary_bvh([&](const size_t tid) { return bool(in_U_mesh[tid]); }, bvh_U);
        } else {
            const double tol = trace_rel_tol * radius;
            std::map<simplex::Edge, Vector3d> traced;
            size_t n_failed = 0;
            // the point of edge (v_out, v_in) where d(x, R) reaches d/2, v_in being in U
            const auto trace_to_U = [&](const size_t v_out, const size_t v_in) -> Vector3d {
                const simplex::Edge e(v_out, v_in);
                const auto it = traced.find(e);
                if (it != traced.end()) {
                    return it->second;
                }
                const Vector3d a = pos(v_out);
                const double L = (pos(v_in) - a).norm();
                const Vector3d dir = (pos(v_in) - a) / L;
                Vector3d p = pos(v_in); // if the trace leaves the edge, U keeps only v_in
                bool on_level = false;
                double t = 0.;
                while (t < L && t >= 0.) {
                    const Vector3d q = a + t * dir;
                    const double g = bvh_distance(bvh_R, q);
                    if (std::abs(g - radius) <= tol) {
                        p = q;
                        on_level = true;
                        break;
                    }
                    t += radius - g;
                }
                if (!on_level) {
                    ++n_failed;
                }
                traced[e] = p;
                return p;
            };
            const auto edge_point = [&](const size_t va, const size_t vb) {
                return in_U[va] ? trace_to_U(vb, va) : trace_to_U(va, vb);
            };

            // the boundary of U_mesh: the faces of whole tets in U not shared with another, and
            // for each cut tet, the part of each face in U (walked corner by corner, fanned into
            // triangles) and the cut polygon (a triangle, or a quad split in two)
            std::vector<Vector3d> triangles;
            const auto add_polygon = [&](const std::vector<Vector3d>& poly) {
                for (size_t k = 1; k + 1 < poly.size(); ++k) {
                    triangles.push_back(poly[0]);
                    triangles.push_back(poly[k]);
                    triangles.push_back(poly[k + 1]);
                }
            };
            for (const size_t tid : tids) {
                if (region[tid] >= 0) {
                    continue;
                }
                const auto vs = oriented_tet_vids(tid);
                if (in_U_mesh[tid]) {
                    for (int j = 0; j < 4; ++j) {
                        const Tuple face_tup = tuple_from_face(tid, j);
                        const auto t_opp = face_tup.switch_tetrahedron(*this);
                        if (t_opp && in_U_mesh[t_opp->tid(*this)]) {
                            continue;
                        }
                        add_polygon(
                            {pos(face_tup.vid(*this)),
                             pos(face_tup.switch_vertex(*this).vid(*this)),
                             pos(face_tup.switch_edge(*this).switch_vertex(*this).vid(*this))});
                    }
                    continue;
                }
                if (!in_U[vs[0]] && !in_U[vs[1]] && !in_U[vs[2]] && !in_U[vs[3]]) {
                    continue;
                }
                for (const auto& f : tet_faces(vs)) {
                    std::vector<Vector3d> part;
                    for (int j = 0; j < 3; ++j) {
                        const size_t va = f[j];
                        const size_t vb = f[(j + 1) % 3];
                        if (in_U[va]) {
                            part.push_back(pos(va));
                        }
                        if (in_U[va] != in_U[vb]) {
                            part.push_back(edge_point(va, vb));
                        }
                    }
                    add_polygon(part);
                }
                // The cut edges, in order around the cut: with four, the one sharing no vertex
                // with the first is opposite it.
                std::vector<std::array<size_t, 2>> cut;
                for (const auto& [a, b] : tet_edges) {
                    if (in_U[vs[a]] != in_U[vs[b]]) {
                        cut.push_back({{vs[a], vs[b]}});
                    }
                }
                if (cut.size() == 4) {
                    for (size_t k = 1; k < 4; ++k) {
                        if (cut[k][0] != cut[0][0] && cut[k][0] != cut[0][1] &&
                            cut[k][1] != cut[0][0] && cut[k][1] != cut[0][1]) {
                            std::swap(cut[k], cut[2]);
                            break;
                        }
                    }
                }
                std::vector<Vector3d> cut_poly;
                for (const auto& [va, vb] : cut) {
                    cut_poly.push_back(edge_point(va, vb));
                }
                add_polygon(cut_poly);
            }
            has_U_mesh = triangles_bvh(triangles, bvh_U);
            logger().info(
                "tight_seal_geom: U traced on {} edges to d(x, R) = d/2 ({} left the edge)",
                traced.size(),
                n_failed);
        }

        // J: the vertices outside U_mesh at distance > d/2 from it (all of them if it is empty)
        std::vector<bool> in_J(vert_capacity(), false);
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            const bool on_U = growth ? bool(in_U[vid]) : bool(on_U_mesh[vid]);
            in_J[vid] = !on_U && (!has_U_mesh || bvh_distance(bvh_U, pos(vid)) > radius);
        }

        // J_mesh: the tets of E whose four vertices are in J. With growth, the tets of E with a
        // vertex in J, cut down below to their part on the J side.
        std::vector<bool> in_J_mesh(tet_capacity(), false);
        for (const size_t tid : tids) {
            const auto vs = oriented_tet_vids(tid);
            int n_in_J = 0;
            for (const size_t v : vs) {
                n_in_J += int(in_J[v]);
            }
            in_J_mesh[tid] = region[tid] < 0 && (growth ? n_in_J > 0 : n_in_J == 4);
        }

        // Connected components of J_mesh; keep the ones that share a face with every R_i (with
        // detected tags: with min_regions R_i or more, which the component is then filled with).
        // Tets connect through a face with a vertex in J, which with growth keeps a part on the J
        // side; without growth every face of J_mesh has.
        std::vector<int> comp_id(tet_capacity(), -1);
        size_t n_components = 0;
        for (const size_t tid : tids) {
            if (!in_J_mesh[tid] || comp_id[tid] != -1) {
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
                    const Tuple face_tup = tuple_from_face(cur, j);
                    if (!in_J[face_tup.vid(*this)] &&
                        !in_J[face_tup.switch_vertex(*this).vid(*this)] &&
                        !in_J[face_tup.switch_edge(*this).switch_vertex(*this).vid(*this)]) {
                        continue;
                    }
                    const auto t_opp = face_tup.switch_tetrahedron(*this);
                    if (!t_opp) {
                        comp.touches_boundary = true;
                        continue;
                    }
                    const size_t nbr = t_opp->tid(*this);
                    if (region[nbr] >= 0) {
                        borders[region[nbr]] = true;
                    }
                    if (!in_J_mesh[nbr] || comp_id[nbr] != -1) {
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
                        "tight_seal_geom: component {} ({} tets) borders tags {}",
                        components_seal.size(),
                        comp.cells.size(),
                        comp_tags);
                }
                components_seal.push_back(std::move(comp));
                components_regions.push_back(std::move(comp_regions));
            }
        }

        if (detected) {
            logger().info(
                "tight_seal_geom: {} of {} unreachable components border {} or more of the "
                "detected tags {}, distance {}",
                components_seal.size(),
                n_components,
                needed,
                tags,
                distance);
        } else {
            logger().info(
                "tight_seal_geom: {} of {} unreachable components border all tags {}, distance {}",
                components_seal.size(),
                n_components,
                tags,
                distance);
        }

        // Growth. Each edge of a sealed component from a vertex in J to one outside is split
        // where it leaves J: where the distance f to U_mesh falls to d/2, found by sphere tracing
        // from the J end. f is 1-Lipschitz, so from a point with f > d/2 the level set is at
        // least f - d/2 away and a step that long along the edge cannot cross it: the trace
        // stops at the first crossing (within the tolerance) and the part of the edge kept is
        // in J. As TopoOffsetTetMesh's marching sphere trace. The component then keeps its tets
        // with no vertex outside J. (No vertex is outside J if U_mesh is empty.)
        if (growth) {
            const double tol = trace_rel_tol * radius;
            Vector3d trace_point;
            Vector3d trace_dir;
            // linear along the edge, zero at the traced point, so the split lands on it
            m_voronoi_split_fn = [&](const Vector3d& p) -> double {
                return (p - trace_point).dot(trace_dir);
            };
            size_t n_traced = 0;
            size_t n_not_split = 0;
            for (ConnectedComponent& comp : components_seal) {
                std::unordered_set<size_t> comp_cells(comp.cells.begin(), comp.cells.end());
                std::vector<simplex::Edge> split_edges;
                for (const size_t tid : comp.cells) {
                    const auto vs = oriented_tet_vids(tid);
                    for (const auto& [a, b] : tet_edges) {
                        if (in_J[vs[a]] != in_J[vs[b]]) {
                            split_edges.emplace_back(vs[a], vs[b]);
                        }
                    }
                }
                vector_unique(split_edges);

                for (const simplex::Edge& e : split_edges) {
                    const auto [e0, e1] = e.vertices();
                    const Vector3d a = pos(in_J[e0] ? e0 : e1);
                    const Vector3d b = pos(in_J[e0] ? e1 : e0);
                    const double L = (b - a).norm();
                    trace_dir = (b - a) / L;
                    // Past the far end: the trace left the edge. At the J end: that end is
                    // already on the level set, and the edge keeps no part in J.
                    bool on_level = false;
                    double t = 0.;
                    while (t < L && t >= 0.) {
                        trace_point = a + t * trace_dir;
                        const double f = bvh_distance(bvh_U, trace_point);
                        if (std::abs(f - radius) <= tol) {
                            on_level = t > 0.;
                            break;
                        }
                        t += f - radius;
                    }
                    if (!on_level) {
                        ++n_not_split;
                        continue;
                    }
                    const size_t v_new =
                        split_tracked(e, comp_cells, [](const size_t, const size_t) {});
                    if (v_new == size_t(-1)) {
                        ++n_not_split;
                        continue;
                    }
                    ++n_traced;
                    if (v_new >= in_J.size()) {
                        in_J.resize(v_new + 1, false);
                    }
                    in_J[v_new] = true; // on the level set, the J side's boundary
                }

                comp.cells.clear();
                comp.volume = 0.;
                for (const size_t tid : comp_cells) {
                    const auto vs = oriented_tet_vids(tid);
                    if (in_J[vs[0]] && in_J[vs[1]] && in_J[vs[2]] && in_J[vs[3]]) {
                        comp.cells.push_back(tid);
                        comp.volume += tet_volume(tid);
                    }
                }
                std::sort(comp.cells.begin(), comp.cells.end());
            }
            m_voronoi_split_fn = nullptr;
            logger().info(
                "tight_seal_geom: growth split {} edges where they leave J, {} not split (the "
                "trace started on the level set, left the edge, or the split was refused)",
                n_traced,
                n_not_split);
        }
    }

    // Voronoi fill, of each component with its regions. A point's region is the one of them
    // nearest to it. An edge whose endpoints are in regions a and b is split where d_a = d_b, by
    // split_adjust_position's bisection.
    const auto nearest_region = [&](const Vector3d& p, const std::vector<size_t>& regions) {
        size_t best = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (const size_t i : regions) {
            const double d = bvh_distance(region_bvhs[i], p);
            if (d < best_d) {
                best = i;
                best_d = d;
            }
        }
        return best;
    };
    size_t split_a = 0;
    size_t split_b = 0;
    m_voronoi_split_fn = [&](const Vector3d& p) -> double {
        return bvh_distance(region_bvhs[split_a], p) - bvh_distance(region_bvhs[split_b], p);
    };

    size_t n_splits = 0;
    for (size_t c = 0; c < components_seal.size(); ++c) {
        const ConnectedComponent& comp = components_seal[c];
        const std::vector<size_t>& regions = components_regions[c];
        // replace the tet's tags of the component's regions with tags[i]; other tags stay
        const auto set_region = [&](const size_t tid, const size_t i) {
            CellTag& tag = m_tet_attribute[tid].tags;
            for (const size_t j : regions) {
                tag.erase(tags[j]);
            }
            tag.insert(tags[i]);
        };

        // the component's tets, kept up to date as its edges are split
        std::unordered_set<size_t> comp_cells(comp.cells.begin(), comp.cells.end());
        std::unordered_map<size_t, size_t> vertex_region;
        for (const size_t tid : comp.cells) {
            const auto vs = oriented_tet_vids(tid);
            set_region(
                tid,
                nearest_region((pos(vs[0]) + pos(vs[1]) + pos(vs[2]) + pos(vs[3])) / 4, regions));
            for (const size_t v : vs) {
                if (vertex_region.count(v) == 0) {
                    vertex_region[v] = nearest_region(pos(v), regions);
                }
            }
        }

        std::vector<simplex::Edge> split_edges;
        for (const size_t tid : comp.cells) {
            const auto vs = oriented_tet_vids(tid);
            for (const auto& [a, b] : tet_edges) {
                const size_t v0 = vs[a];
                const size_t v1 = vs[b];
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
            const size_t v_new =
                split_tracked(e, comp_cells, [&](const size_t tid, const size_t end_vid) {
                    set_region(tid, vertex_region.at(end_vid));
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
