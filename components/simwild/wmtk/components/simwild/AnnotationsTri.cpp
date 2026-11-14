#include "SimWildMeshTri.hpp"
#include "TightSealGeom.hpp"

#include <wmtk/utils/SimplicialEmbedding.hpp>

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

            const size_t v_new = m_last_split_vertex.local();
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
    const ExprPtr& input_selection,
    const std::vector<ExprPtr>& fill_selections,
    const std::vector<CellTag>& fill_tags,
    const double distance,
    const double trace_rel_tol,
    const int64_t min_regions,
    const bool sanity,
    const int64_t max_preprocessing_rounds,
    const bool material_interface_fill)
{
    if (material_interface_fill) {
        log_and_throw_error(
            "tight_seal_geom: tight_seal_geom_EXPERIMENTAL_material_interface_fill is 3D only "
            "for now");
    }
    if (!input_selection) {
        log_and_throw_error("tight_seal_geom needs an input selection");
    }
    if (fill_selections.size() < 2) {
        log_and_throw_error(
            "tight_seal_geom needs two or more fill selections. Input was {}",
            fill_selections.size());
    }
    if (fill_tags.size() != fill_selections.size()) {
        log_and_throw_error(
            "tight_seal_geom needs one fill tag set per fill selection: {} selections, {} tag "
            "sets",
            fill_selections.size(),
            fill_tags.size());
    }
    if (min_regions < 2) {
        log_and_throw_error(
            "tight_seal_geom: min_regions must be at least 2. Input was {}",
            min_regions);
    }
    if (max_preprocessing_rounds < 0) {
        log_and_throw_error(
            "tight_seal_geom: max_preprocessing_rounds must be at least 0. Input was {}",
            max_preprocessing_rounds);
    }
    // the colors of the fill, one per fill selection, and how many a component must border to be
    // filled
    const size_t n = fill_selections.size();
    const size_t needed = size_t(min_regions);
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

    // R, the region closed to find the join region: the faces the input selection holds for.
    // A face's color for the fill: the one fill selection it satisfies, -1 if none, -2 if
    // several. Both by tag, so they hold for the faces the splits below make.
    const auto in_R = [&](const size_t fid) {
        return input_selection->eval(m_face_attribute[fid].tags);
    };
    const std::function<int(size_t)> color_of = [&](const size_t fid) {
        const CellTag& tag = m_face_attribute[fid].tags;
        int found = -1;
        for (size_t k = 0; k < n; ++k) {
            if (fill_selections[k]->eval(tag)) {
                if (found >= 0) {
                    return -2;
                }
                found = int(k);
            }
        }
        return found;
    };
    bool can_seal = false;
    for (const Tuple& t : get_faces()) {
        if (in_R(t.fid(*this))) {
            can_seal = true;
            break;
        }
    }

    // 0. R, the union of the regions, is made simplicially embedded
    // (wmtk/utils/SimplicialEmbedding.hpp): each face outside R whose three edges lie on R is
    // split at its centroid, then each edge outside R joining two vertices of R at its midpoint.
    // Then no face outside R has all its corners on R: a gap one face wide gets a vertex off R,
    // whose distances decide whether the gap is in J. (Every vertex of R counts as inside J, so
    // without the embedding such a face would be in J at any distance.)
    if (can_seal) {
        const wmtk::utils::TriComplex R_complex{
            [&](const size_t vid) {
                for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
                    if (in_R(fid)) {
                        return true;
                    }
                }
                return false;
            },
            [&](const Tuple& e) {
                if (in_R(e.fid(*this))) {
                    return true;
                }
                const auto opp = e.switch_face(*this);
                return opp.has_value() && in_R(opp->fid(*this));
            },
            [&](const Tuple& f) { return in_R(f.fid(*this)); }};
        const wmtk::utils::SimplicialEmbeddingStats emb =
            wmtk::utils::simplicial_embedding(*this, R_complex);
        logger().info(
            "tight_seal_geom: simplicial embedding of the regions: split {} faces and {} edges",
            emb.faces_split,
            emb.edges_split);
        if (emb.refused > 0) {
            logger().warn(
                "tight_seal_geom: {} splits of the simplicial embedding were refused; a face "
                "outside the regions may still have all its corners on them",
                emb.refused);
        }
    }

    std::vector<size_t> fids;
    std::vector<int> region(tri_capacity(), -1);
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        fids.push_back(fid);
        region[fid] = in_R(fid) ? 0 : -1;
    }

    // m_voronoi_split_fn places the vertex of each split below (split_adjust_position)
    tight_seal::ClearOnExit<std::function<double(const Vector2d&)>> clear_voronoi_split{
        m_voronoi_split_fn};

    // filled[fid]: the fill set the face's region. A split's children inherit it from
    // their parent, so it stays right whichever component's split cuts the face. For the sanity
    // check.
    std::vector<char> filled(tri_capacity(), 0);

    std::vector<SimpleBVH::BVH> color_bvhs(n);
    std::vector<ConnectedComponent> components_seal;
    // the colors each sealed component is filled with, in increasing order
    std::vector<std::vector<size_t>> components_colors;
    // in_J[id]: the face is in the join region J (in any component, filled or not)
    std::vector<char> in_J;
    // the faces beside J that satisfy several fill selections
    std::vector<size_t> ambiguous;
    if (!can_seal) {
        logger().warn(
            "tight_seal_geom: no face satisfies the input selection {}, nothing to seal",
            input_selection->to_string());
    } else {
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

        // The tolerance of both sphere traces and of the vertices' labels below: trace_rel_tol x
        // the mesh's bounding box diagonal, independent of the distance (tight seal v2 note,
        // section 2).
        Vector2d box_lo = Vector2d::Constant(std::numeric_limits<double>::infinity());
        Vector2d box_hi = -box_lo;
        for (const Tuple& t : get_vertices()) {
            box_lo = box_lo.cwiseMin(pos(t.vid(*this)));
            box_hi = box_hi.cwiseMax(pos(t.vid(*this)));
        }
        const double diagonal = (box_hi - box_lo).norm();
        const double tol = trace_rel_tol * diagonal;
        logger().info(
            "tight_seal_geom: tolerance {} ({} x the bounding box diagonal {})",
            tol,
            trace_rel_tol,
            diagonal);

        // 1. The dilation surface S, the level set d(x, R) = d/2 (the boundary of R + B),
        // traced on the edges that cross it and joined in each face. The closing is the part
        // of R + B at least d/2 from S, so J ends where d(x, S) falls to d/2 (to d/2 + tol, with
        // the tolerance of step 2).
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
            "tight_seal_geom: dilation surface found on {} edges where d(x, I) = d/2 ({} with no "
            "crossing found, their far end taken)",
            n_S_traced,
            n_S_failed);
        const std::function<double(const Vector2d&)> dist_S = [&](const Vector2d& p) {
            return has_S ? tight_seal::bvh_distance(bvh_S, p)
                         : std::numeric_limits<double>::infinity();
        };

        // 2. The vertices' labels and the marching of J (tight seal v2 note, section 2; R and S
        // here are the note's input region I and dilated surface D). Every vertex within d/2 of
        // R, R's own included, is IN if d(v, S) >= d/2 + tol, else OUT; the vertices beyond S are
        // OUT. The faces whose vertices are all IN are marked, R's included. Each edge from an IN
        // vertex to an OUT one is split where d(x, S) falls to d/2 + tol, the labels' level
        // (sphere traced from the IN end, then bisected; tight_seal::level_crossing), and of the
        // faces it makes, those touching its IN end are marked and the others not; the new vertex
        // takes no label. R's faces may thus be split. J is then the marked faces outside R.
        std::vector<char> inside(vert_capacity(), 0);
        size_t n_in = 0;
        // DEBUG_output: each vertex's label, 0 OUT, 1 IN (-1 for a vertex a split makes, which
        // takes none), for the join-region frame
        std::vector<int> label;
        if (m_params.debug_output) {
            label.assign(vert_capacity(), 0);
        }
        for (const Tuple& t : get_vertices()) {
            const size_t vid = t.vid(*this);
            inside[vid] = dR[vid] < radius && dist_S(pos(vid)) >= radius + tol;
            n_in += inside[vid];
            if (m_params.debug_output) {
                label[vid] = inside[vid];
            }
        }
        logger().info(
            "tight_seal_geom: {} vertices within d/2 of I (I's own included) are IN, at least "
            "d/2 + tol from D; the others are OUT",
            n_in);
        std::unordered_set<size_t> marked;
        std::vector<simplex::Edge> split_edges;
        for (const size_t fid : fids) {
            const auto vs = oriented_tri_vids(fid);
            if (inside[vs[0]] && inside[vs[1]] && inside[vs[2]]) {
                marked.insert(fid);
            }
            for (int j = 0; j < 3; ++j) {
                if (inside[vs[j]] != inside[vs[(j + 1) % 3]]) {
                    split_edges.emplace_back(vs[j], vs[(j + 1) % 3]);
                }
            }
        }
        vector_unique(split_edges);
        const size_t n_marked = marked.size();

        Vector2d trace_point;
        Vector2d trace_dir;
        // linear along the edge, zero at the traced point, so the split lands on it
        m_voronoi_split_fn = [&](const Vector2d& p) -> double {
            return (p - trace_point).dot(trace_dir);
        };
        size_t n_split = 0;
        size_t n_not_split = 0;
        std::unordered_set<size_t> unmarked; // the tracked split's set; marks are set below
        for (const simplex::Edge& e : split_edges) {
            const auto [e0, e1] = e.vertices();
            const size_t in_end = inside[e0] ? e0 : e1;
            const Vector2d a = pos(in_end);
            const Vector2d b = pos(in_end == e0 ? e1 : e0);
            trace_dir = (b - a) / (b - a).norm();
            // Split where d(x, S) falls to d/2 + tol, the labels' level: an IN end is at least
            // there, an OUT one below it or beyond S.
            const std::optional<Vector2d> crossing =
                tight_seal::level_crossing(dist_S, a, b, radius + tol, tol, false);
            if (!crossing || !((*crossing - a).dot(trace_dir) > 0.)) {
                ++n_not_split;
                continue;
            }
            trace_point = *crossing;
            const std::vector<size_t> parents = get_incident_fids_for_edge(e0, e1);
            const size_t v_new = tight_seal_split_tracked(e, unmarked, filled);
            if (v_new == size_t(-1)) {
                ++n_not_split; // refused: the faces and their marks are as they were
                continue;
            }
            ++n_split;
            for (const size_t id : parents) {
                marked.erase(id);
            }
            for (const size_t id : get_one_ring_fids_for_vertex(v_new)) {
                const auto vs = oriented_tri_vids(id);
                if (std::find(vs.begin(), vs.end(), in_end) != vs.end()) {
                    marked.insert(id);
                } else {
                    marked.erase(id);
                }
            }
            if (m_params.debug_output) {
                if (v_new >= label.size()) {
                    label.resize(v_new + 1, 0);
                }
                label[v_new] = -1;
            }
        }
        m_voronoi_split_fn = nullptr;

        // J leaves out R's faces, by tag (a split's children keep it)
        size_t n_left_out = 0;
        for (auto it = marked.begin(); it != marked.end();) {
            if (in_R(*it)) {
                it = marked.erase(it);
                ++n_left_out;
            } else {
                ++it;
            }
        }
        std::vector<size_t> J_faces(marked.begin(), marked.end());
        std::sort(J_faces.begin(), J_faces.end());
        in_J.assign(tri_capacity(), 0);
        for (const size_t id : J_faces) {
            in_J[id] = 1;
        }
        logger().info(
            "tight_seal_geom: marked {} faces with every vertex IN (I's included), split {} edges "
            "where d(x, D) falls to d/2 + tol and marked their faces at the IN end ({} not split: "
            "no crossing found, or the split was refused), then left out {} faces of I; J holds "
            "{} faces",
            n_marked,
            n_split,
            n_not_split,
            n_left_out,
            J_faces.size());


        // 3. Connected components of J (through edges). A component borders a color if one of
        // its faces shares an edge with a face outside J of that color (a face of J
        // is no color, whatever its tags); it is filled, with the colors it borders, if it borders
        // min_regions of them or more. A face beside J satisfying several fill selections is an
        // error, reported below.
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
                    if (!in_J[nbr]) {
                        const int k = color_of(nbr);
                        if (k >= 0) {
                            borders[size_t(k)] = true;
                        } else if (k == -2) {
                            ambiguous.push_back(nbr);
                        }
                        continue;
                    }
                    if (comp_id[nbr] != -1) {
                        continue;
                    }
                    comp_id[nbr] = int(n_components);
                    comp.cells.push_back(nbr);
                }
            }
            ++n_components;
            std::vector<size_t> comp_colors;
            for (size_t k = 0; k < n; ++k) {
                if (borders[k]) {
                    comp_colors.push_back(k);
                }
            }
            if (comp_colors.size() >= needed) {
                logger().info(
                    "tight_seal_geom: component {} ({} faces) borders colors {}",
                    components_seal.size(),
                    comp.cells.size(),
                    comp_colors);
                std::sort(comp.cells.begin(), comp.cells.end());
                components_seal.push_back(std::move(comp));
                components_colors.push_back(std::move(comp_colors));
            }
        }
        logger().info(
            "tight_seal_geom: J has {} components; {} of them border {} colors or more and "
            "are filled",
            n_components,
            components_seal.size(),
            needed);

        // DEBUG_output: the mesh after J is found, before anything is filled. Vertex field
        // join_label: 0 OUT, 1 IN, -1 made by a split (step 2). Cell fields: join_region marks
        // J's faces, join_component numbers J's components (as above; -1 outside J), and
        // join_filled marks the components that are filled. Written before the boundary check
        // below, which may throw.
        if (m_params.debug_output) {
            std::vector<double> join(tri_capacity(), 0.), component(tri_capacity(), -1.),
                to_fill(tri_capacity(), 0.);
            for (const size_t id : J_faces) {
                join[id] = 1.;
                component[id] = comp_id[id];
            }
            for (const ConnectedComponent& comp : components_seal) {
                for (const size_t id : comp.cells) {
                    to_fill[id] = 1.;
                }
            }
            write_vtu(
                fmt::format("debug_{}_join_regions", m_debug_print_counter++),
                nullptr,
                {{"join_region", join}, {"join_component", component}, {"join_filled", to_fill}},
                {{"join_label", std::vector<double>(label.begin(), label.end())}});
        }

        // The fill selections must not be ambiguous beside J, in any component.
        if (!ambiguous.empty()) {
            std::sort(ambiguous.begin(), ambiguous.end());
            ambiguous.erase(std::unique(ambiguous.begin(), ambiguous.end()), ambiguous.end());
            const CellTag& tag = m_face_attribute[ambiguous.front()].tags;
            std::vector<std::string> names;
            for (const int64_t t : tag) {
                const auto it = m_tag_id_to_name.find(t);
                names.push_back(it != m_tag_id_to_name.end() ? it->second : std::to_string(t));
            }
            std::vector<std::string> satisfied;
            for (size_t k = 0; k < n; ++k) {
                if (fill_selections[k]->eval(tag)) {
                    satisfied.push_back(fill_selections[k]->to_string());
                }
            }
            log_and_throw_error(
                "tight_seal_geom: the fill selections are ambiguous: {} faces beside the "
                "join region satisfy more than one of them; one with tags {} satisfies {}",
                ambiguous.size(),
                names,
                satisfied);
        }

        // J may not touch the mesh's boundary at all, not even at a vertex, in any component: the
        // topology of a region that reaches the boundary is much harder to reason about.
        const size_t n_on_boundary = tight_seal_cells_on_boundary(J_faces);
        if (n_on_boundary > 0) {
            size_t n_to_fill = 0;
            for (const ConnectedComponent& comp : components_seal) {
                n_to_fill += tight_seal_cells_on_boundary(comp.cells);
            }
            log_and_throw_error(
                "tight_seal_geom: the join region touches the mesh boundary: {} of its {} faces "
                "have a vertex on it, {} of them in parts to fill. Keep the regions more than "
                "distance / 2 from the boundary",
                n_on_boundary,
                J_faces.size(),
                n_to_fill);
        }

        // The distance to each color the components to fill border: from outside its faces,
        // the distance to their boundary. A face of J is no color, whatever its tags.
        std::vector<char> color_used(n, 0);
        for (const std::vector<size_t>& colors : components_colors) {
            for (const size_t k : colors) {
                color_used[k] = 1;
            }
        }
        std::vector<size_t> all_ids;
        for (const Tuple& t : get_faces()) {
            all_ids.push_back(t.fid(*this));
        }
        for (size_t k = 0; k < n; ++k) {
            if (color_used[k]) {
                tight_seal_boundary_bvh(
                    all_ids,
                    [&](const size_t id) { return !in_J[id] && color_of(id) == int(k); },
                    color_bvhs[k]);
            }
        }
    }

    // Each component is preprocessed, then split as its faces' own labels of their corners
    // require, and each face then takes a color from its corners' labels and gets that color's
    // fill tags added to its own (tight_seal_split_fill).
    std::vector<CellTag> tags_before_fill;
    tight_seal_split_fill(
        components_seal,
        components_colors,
        fill_tags,
        color_bvhs,
        color_of,
        in_J,
        max_preprocessing_rounds,
        filled,
        tags_before_fill);

    // Topology check of every tag the fill changed, before the fill and after it: warns if a tag
    // gained or lost non-manifold vertices, or the fill made an island of it or merged two of
    // its components.
    tight_seal_topology_check(filled, tags_before_fill);

    // Sanity checks, on the output alone rather than on the components above: no face is
    // inverted; each filled face carries one of the fill tag sets; and each part of the filled
    // faces, connected through edges, borders two colors or more. A part bordering a
    // single color would be a concave feature of that body, filled.
    if (sanity) {
        tight_seal_geom_sanity(fill_tags, color_of, filled);
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
