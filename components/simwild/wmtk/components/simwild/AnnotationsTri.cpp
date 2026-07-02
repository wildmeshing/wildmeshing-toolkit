#include "SimWildMeshTri.hpp"

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
                vertices.push_back(m_vertex_attribute.at(v0).m_pos);
                vertices.push_back(m_vertex_attribute.at(v1).m_pos);
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
        const Vector2d& p0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2d& p1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2d& p2 = m_vertex_attribute[vs[2]].m_pos;
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

    // seal holes
    for (const ConnectedComponent& hole : components) {
        std::vector<simplex::Edge> split_edges;
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
                const double d0 = m_voronoi_split_fn(m_vertex_attribute.at(v0).m_pos);
                const double d1 = m_voronoi_split_fn(m_vertex_attribute.at(v1).m_pos);
                // only split edges if their endpoints aren't already on the surface
                if ((d0 < -1e-20 && d1 > 1e-20) || (d1 < -1e-20 && d0 > 1e-20)) {
                    split_edges.emplace_back(v0, v1);
                }
            }
        }
        vector_unique(split_edges);

        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }

        std::vector<Tuple> new_tris;
        for (const simplex::Edge& e : split_edges) {
            const auto [t, eid] = tuple_from_edge(e.vertices());
            if (!split_edge(t, new_tris)) {
                continue;
            }
            for (const Tuple& f_tup : new_tris) {
                const size_t fid = f_tup.fid(*this);
                const auto vs = oriented_tri_vids(fid);
                // find vertex from splitted edge
                size_t vid = -1;
                for (const size_t v : vs) {
                    if (e.vertices()[0] == v || e.vertices()[1] == v) {
                        vid = v;
                        break;
                    }
                }
                if (vid == -1) {
                    log_and_throw_error("Could not find edge-vertex after split.");
                }

                const Vector2d& p = m_vertex_attribute[vid].m_pos;
                const double d = m_voronoi_split_fn(p);
                auto& tag = m_face_attribute[fid].tags;
                if (d < 0) {
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

void SimWildMeshTri::resolve_intersections(const std::vector<CellTag>& intersecting_tags)
{
    for (const CellTag& tag_set : intersecting_tags) {
        if (tag_set.size() != 2) {
            log_and_throw_error(
                "Can only resolve intersections between two tags at once. Input was {}",
                tag_set);
        }
        std::vector<ConnectedComponent> components = compute_connected_components(tag_set);
        if (components.empty()) {
            logger().info("No intersections in between tags {}", tag_set);
            continue;
        }
        logger().info("Resolve {} intersections in between tags {}", components.size(), tag_set);

        // remove tags from components
        for (const ConnectedComponent& comp : components) {
            for (const size_t fid : comp.cells) {
                auto& tag = m_face_attribute[fid].tags;
                for (const size_t tt : tag_set) {
                    tag.erase(tt);
                }
            }
        }

        std::vector<CellTag> tag_vec;
        tag_vec.reserve(2);
        for (const size_t tag : tag_set) {
            CellTag ct;
            ct.insert(tag);
            tag_vec.push_back(ct);
        }
        seal_connected_components(tag_vec, components);
    }

    // report other intersections
    std::map<size_t, std::set<size_t>> intersections;
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
