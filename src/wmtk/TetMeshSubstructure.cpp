#include <wmtk/TetMesh.h>

#include <queue>

namespace wmtk {

simplex::SimplexCollection TetMesh::get_surface_faces_for_vertex(const size_t vid) const
{
    using namespace simplex;

    SimplexCollection sc;

    if (!vertex_is_on_surface(vid)) {
        // no face can be on the surface if the vertex is not on the surface
        return sc;
    }

    const auto tids = get_one_ring_tids_for_vertex(vid);
    for (const size_t tid : tids) {
        const Tet tet = simplex_from_tet(tid);
        const Face f_opp = tet.opposite_face(vid);
        const auto& vs = f_opp.vertices();
        for (size_t i = 0; i < 3; ++i) {
            size_t j = (i + 1) % 3;
            if (!vertex_is_on_surface(vs[i]) || !vertex_is_on_surface(vs[j])) {
                // face cannot be on the surface
                continue;
            }

            const auto [f_tuple, fid] = tuple_from_face({{vid, vs[i], vs[j]}});

            if (face_is_on_surface(fid)) {
                sc.add(Face(vid, vs[i], vs[j]));
            }
        }
    }
    sc.sort_and_clean();

    return sc;
}

simplex::SimplexCollection TetMesh::get_surface_faces_for_edge(
    const std::array<size_t, 2>& vids) const
{
    const size_t& v0 = vids[0];
    const size_t& v1 = vids[1];

    simplex::SimplexCollection sc;

    // vertices must be on the surface, otherwise edge cannot be on the surface
    if (!vertex_is_on_surface(v0) || !vertex_is_on_surface(v1)) {
        return sc;
    }

    const auto n12_locs = get_incident_tids_for_edge(v0, v1);

    std::vector<size_t> link_vs;
    link_vs.reserve(n12_locs.size());
    for (const size_t& tid : n12_locs) {
        const auto vs = oriented_tet_vids(tid);
        for (int i = 0; i < 4; ++i) {
            if (vs[i] != v0 && vs[i] != v1) {
                link_vs.push_back(vs[i]);
            }
        }
    }
    wmtk::vector_unique(link_vs);

    for (const size_t v2 : link_vs) {
        simplex::Face f(v0, v1, v2);
        const auto [f_tuple, fid] = tuple_from_face(f);
        if (face_is_on_surface(fid)) {
            sc.add(f);
        }
    }

    sc.sort_and_clean();

    return sc;
}

size_t TetMesh::get_num_surface_faces_for_edge(const std::array<size_t, 2>& vids) const
{
    const size_t& v0 = vids[0];
    const size_t& v1 = vids[1];

    // vertices must be on the surface, otherwise edge cannot be on the surface
    if (!vertex_is_on_surface(v0) || !vertex_is_on_surface(v1)) {
        return 0;
    }

    const auto n12_locs = get_incident_tids_for_edge(v0, v1);

    std::vector<size_t> link_vs;
    link_vs.reserve(n12_locs.size());
    for (const size_t& tid : n12_locs) {
        const auto vs = oriented_tet_vids(tid);
        for (int i = 0; i < 4; ++i) {
            if (vs[i] != v0 && vs[i] != v1) {
                link_vs.push_back(vs[i]);
            }
        }
    }
    wmtk::vector_unique(link_vs);

    size_t surface_count = 0;
    for (const size_t v2 : link_vs) {
        const auto [f_tuple, fid] = tuple_from_face({{v0, v1, v2}});
        if (face_is_on_surface(fid)) {
            ++surface_count;
        }
    }

    return surface_count;
}

size_t TetMesh::compute_vertex_order(const size_t vid) const
{
    if (!vertex_is_on_surface(vid)) {
        return 0;
    }

    size_t vertex_order = 1;

    // check incident order-2 edges
    const auto vvs = get_one_ring_vids_for_vertex(vid);
    std::map<size_t, size_t> edge_orders_count;
    for (const size_t v : vvs) {
        const size_t nf = get_num_surface_faces_for_edge({v, vid});
        if (nf == 0 || nf == 2) {
            continue;
        }
        if (edge_orders_count.count(nf) == 0) {
            edge_orders_count[nf] = 1;
        } else {
            ++edge_orders_count[nf];
        }
    }
    if (edge_orders_count.size() != 0) {
        if (edge_orders_count.size() == 1 && edge_orders_count.begin()->second == 2) {
            /**
             * The vertex is either on a boundary or on a non-manifold edge and has two edges
             * incident that are of the same type.
             */
            vertex_order = 2;
            // do not return here, vertex could still be non-manifold
        } else {
            /**
             * The vertex is either at the end of a non-manifold edge or the number of incident
             * faces is different for the incident edges. In both cases, the vertex must be order 3.
             */
            vertex_order = 3;
            // highest possible order reached; we can return here
            return vertex_order;
        }
    }

    /**
     * Check if vertex is non-manifold:
     * All faces must be reachable by iterating through edges, starting from one face.
     */

    const auto face_collection = get_surface_faces_for_vertex(vid);
    const auto& faces = face_collection.faces();

    if (faces.empty()) {
        log_and_throw_error("Vertex must have incindent surface faces");
    }

    std::set<simplex::Face> found_faces;
    std::queue<simplex::Face> q;

    q.push(faces[0]);
    found_faces.insert(faces[0]);

    while (!q.empty()) {
        const simplex::Face f = q.front();
        q.pop();

        const auto& vs = f.vertices();
        // navigate through all edges to find other faces
        for (size_t i = 0; i < 3; ++i) {
            const size_t j = (i + 1) % 3;
            const simplex::Edge e(vs[i], vs[j]);
            const auto neighs = face_collection.faces_with_edge(e);
            for (const simplex::Face& n : neighs) {
                if (found_faces.count(n) == 0) {
                    q.push(n);
                    found_faces.insert(n);
                }
            }
        }
    }

    if (found_faces.size() != faces.size()) {
        // surface is non-manifold in this vertex
        vertex_order = 3;
    }

    return vertex_order;
}

size_t TetMesh::get_order_of_edge(const std::array<size_t, 2>& vids) const
{
    const size_t surface_count = get_num_surface_faces_for_edge(vids);
    if (surface_count == 0) {
        // edge is not on the surface
        return 0;
    }
    if (surface_count == 2) {
        // edge is on the surface
        return 1;
    }
    // edge is on the surface boundary or non-manifold
    return 2;
}

bool TetMesh::substructure_link_condition(const Tuple& e_tuple) const
{
    const size_t u_id = e_tuple.vid(*this);
    const size_t v_id = e_tuple.switch_vertex(*this).vid(*this);

    using namespace simplex;

    const size_t edge_order = get_order_of_edge({{u_id, v_id}});
    const size_t u_order = get_order_of_vertex(u_id);
    const size_t v_order = get_order_of_vertex(v_id);

    // If the edge is lower order than both vertices, we know for sure that this edge must not
    // be collapsed. Example: edge in space (order 0) connecting two surfaces (order 1).
    // This check also covers the case that both vertices are order 3
    if (edge_order < u_order && edge_order < v_order) {
        return false;
    }

    const auto u_locs = get_one_ring_tids_for_vertex(u_id);
    const auto v_locs = get_one_ring_tids_for_vertex(v_id);
    const auto e_locs = set_intersection(u_locs, v_locs);

    SimplexCollection link_u_0;
    SimplexCollection link_u_1;
    SimplexCollection link_v_0;
    SimplexCollection link_v_1;
    SimplexCollection link_e_0;
    SimplexCollection link_e_1;

    constexpr size_t w_id = -1; // dummy vertex
    const Vertex w(w_id);

    const SimplexCollection u_surface_faces = get_surface_faces_for_vertex(u_id);
    const SimplexCollection v_surface_faces = get_surface_faces_for_vertex(v_id);
    const SimplexCollection e_surface_faces =
        SimplexCollection::get_intersection(u_surface_faces, v_surface_faces);

    // vertex u links
    {
        const Vertex u(u_id);

        link_u_0.reserve_faces(u_locs.size());
        link_u_0.reserve_edges(u_locs.size() * 3);
        link_u_0.reserve_vertices(u_locs.size() * 3);
        for (const size_t tid : u_locs) {
            const Tet tet = simplex_from_tet(tid);
            const Face f = tet.opposite_face(u);
            link_u_0.add_with_faces(f);
        }

        link_u_1.reserve_edges(u_surface_faces.faces().size());
        link_u_1.reserve_vertices(u_surface_faces.faces().size() * 2);

        SimplexCollection order2_edges;
        for (const Face& f : u_surface_faces.faces()) {
            const Tet tw(f, w_id);

            const Face fw = tw.opposite_face(u);
            link_u_0.add_with_faces(fw);

            const Edge e_opp = f.opposite_edge(u);
            link_u_1.add_with_faces(e_opp);

            const auto& [ev0, ev1] = e_opp.vertices();

            // collect order 2 edges
            if (u_order > 1 || get_order_of_vertex(ev0) > 1) {
                const Edge e0(u_id, ev0);
                if (get_order_of_edge(e0.vertices()) > 1) {
                    order2_edges.add(e0);
                }
            }
            if (u_order > 1 || get_order_of_vertex(ev1) > 1) {
                const Edge e1(u_id, ev1);
                if (get_order_of_edge(e1.vertices()) > 1) {
                    order2_edges.add(e1);
                }
            }
        }
        order2_edges.sort_and_clean();
        for (const Edge& e : order2_edges.edges()) {
            const Face fw(e, w_id);
            const Edge ew = fw.opposite_edge(u);
            link_u_0.add_with_faces(ew);
            link_u_1.add_with_faces(ew);
        }
        link_u_0.sort_and_clean();
        link_u_1.sort_and_clean();
    }
    // vertex v links
    {
        const Vertex v(v_id);

        link_v_0.reserve_faces(v_locs.size());
        link_v_0.reserve_edges(v_locs.size() * 3);
        link_v_0.reserve_vertices(v_locs.size() * 3);
        for (const size_t tid : v_locs) {
            const Tet tet = simplex_from_tet(tid);
            const Face f = tet.opposite_face(v);
            link_v_0.add_with_faces(f);
        }

        link_v_1.reserve_edges(v_surface_faces.faces().size());
        link_v_1.reserve_vertices(v_surface_faces.faces().size() * 2);

        SimplexCollection order2_edges;
        for (const Face& f : v_surface_faces.faces()) {
            const Tet tw(f, w_id);

            const Face fw = tw.opposite_face(v);
            link_v_0.add_with_faces(fw);

            const Edge e_opp = f.opposite_edge(v);
            link_v_1.add_with_faces(e_opp);

            const auto& [ev0, ev1] = e_opp.vertices();

            // collect order 2 edges
            if (v_order > 1 || get_order_of_vertex(ev0) > 1) {
                const Edge e0(v_id, ev0);
                if (get_order_of_edge(e0.vertices()) > 1) {
                    order2_edges.add(e0);
                }
            }
            if (v_order > 1 || get_order_of_vertex(ev1) > 1) {
                const Edge e1(v_id, ev1);
                if (get_order_of_edge(e1.vertices()) > 1) {
                    order2_edges.add(e1);
                }
            }
        }
        order2_edges.sort_and_clean();
        for (const Edge& e : order2_edges.edges()) {
            const Face fw(e, w_id);
            const Edge ew = fw.opposite_edge(v);
            link_v_0.add_with_faces(ew);
            link_v_1.add_with_faces(ew);
        }
        link_v_0.sort_and_clean();
        link_v_1.sort_and_clean();
    }
    // edge links
    {
        const Edge e(u_id, v_id);

        link_e_0.reserve_edges(e_locs.size());
        link_e_0.reserve_vertices(e_locs.size() * 2);
        for (const size_t tid : e_locs) {
            const Tet tet = simplex_from_tet(tid);
            const Edge e_opp = tet.opposite_edge(e);
            link_e_0.add_with_faces(e_opp);
        }

        link_e_1.reserve_vertices(e_surface_faces.size());

        for (const Face& f : e_surface_faces.faces()) {
            const Tet tw(f, w_id);
            const Edge e_opp = tw.opposite_edge(e);
            link_e_0.add_with_faces(e_opp);
            link_e_1.add(f.opposite_vertex(e));
        }

        if (edge_order > 1) {
            link_e_0.add(w);
            link_e_1.add(w);
        }
        link_e_0.sort_and_clean();
        link_e_1.sort_and_clean();
    }

    const auto link_uv_0 = SimplexCollection::get_intersection(link_u_0, link_v_0);
    if (link_uv_0 != link_e_0) {
        return false;
    }
    const auto link_uv_1 = SimplexCollection::get_intersection(link_u_1, link_v_1);
    if (link_uv_1 != link_e_1) {
        return false;
    }

    return true;
}

} // namespace wmtk
