#include "ManExtractMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/orientable_patches.h>
#include <igl/predicates/predicates.h>
#include <igl/remove_unreferenced.h>
#include <igl/winding_number.h>
#include <igl/write_triangle_mesh.h>
#include <spdlog/fmt/bundled/format.h>
#include <spdlog/fmt/ostr.h>
#include <tbb/concurrent_vector.h>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
// clang-format on

#include <paraviewo/VTUWriter.hpp>

#include <cmath>
#include <limits>


namespace wmtk::components::manifold_extraction {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}

// assumes tag has been found. won't be called otherwise
void ManExtractMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixSi& T_tag,
    const MatrixXd& V_env,
    const MatrixXi& F_env)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tag.rows());
    assert((V_env.rows() == 0) || (V_env.cols() == 3));
    assert((F_env.rows() == 0) || (F_env.cols() == 3));

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(6 * T.rows());
    m_face_attribute.m_attributes.resize(4 * T.rows());
    m_tet_attribute.m_attributes.resize(T.rows());

    m_tags_count = T_tag.cols();
    if (V_env.rows() > 0) {
        m_has_envelope = true;
        m_V_envelope = V_env;
        m_F_envelope = F_env;
    }

    // initialize tet tags. if tet tag is subset of in_tag, then in_out=true
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        for (int j = 0; j < m_tags_count; j++) {
            if (T_tag.coeff(t_id, j) == 1) {
                m_tet_attribute[t_id].tag.insert(j);
            }
        }
    }

    // set in_out for tets
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (any_tag_present(m_tet_attribute[t_id].tag, m_params.in_tag)) {
            m_tet_attribute[t_id].in_out = true;
        }
    }

    // initialize vertex coords
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        m_vertex_attribute[v.vid(*this)].m_posf = V.row(v.vid(*this)); // position
    }

    // label simplices on surface
    label_surface_simplices(false);
}


void ManExtractMesh::label_surface_simplices(bool need_to_clear)
{
    auto faces = get_faces();

    if (need_to_clear) {
        // reset all simplices
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            m_vertex_attribute[v.vid(*this)].on_surf = false;
        }
        verts.clear();

        auto edges = get_edges();
        for (const Tuple& e : edges) {
            m_edge_attribute[e.eid(*this)].on_surf = false;
        }
        edges.clear();

        for (const Tuple& f : faces) {
            m_face_attribute[f.fid(*this)].on_surf = false;
        }
    }

    // actually update
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);

        // determine if on surface
        bool on_surf = false;
        if (m_tet_attribute[f.tid(*this)].in_out) {
            auto other = f.switch_tetrahedron(*this);
            if (other) {
                if (!m_tet_attribute[other.value().tid(*this)].in_out) {
                    on_surf = true;
                }
            } else {
                on_surf = true;
            }
        } else {
            auto other = f.switch_tetrahedron(*this);
            if (other) {
                if (m_tet_attribute[other.value().tid(*this)].in_out) {
                    on_surf = true;
                }
            }
        }

        // update
        if (on_surf) {
            m_face_attribute[f_id].on_surf = true;
            // propagate to children
            m_edge_attribute[f.eid(*this)].on_surf = true;
            m_edge_attribute[f.switch_edge(*this).eid(*this)].on_surf = true;
            m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].on_surf = true;
            m_vertex_attribute[f.vid(*this)].on_surf = true;
            m_vertex_attribute[f.switch_vertex(*this).vid(*this)].on_surf = true;
            m_vertex_attribute[f.switch_edge(*this).switch_vertex(*this).vid(*this)].on_surf = true;
        }
    }
}


std::pair<size_t, size_t> ManExtractMesh::label_non_manifold()
{
    // label nm edges
    size_t nm_edge_count = 0;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].on_surf &&
            !edge_is_manifold(e)) { // edge is part of input mesh and not manifold
            m_edge_attribute[e.eid(*this)].label = 1;
            m_vertex_attribute[e.vid(*this)].label = 1;
            m_vertex_attribute[e.switch_vertex(*this).vid(*this)].label = 1;
            nm_edge_count++;
        }
    }

    // label nm vertices
    auto vertices = get_vertices();
    for (const Tuple& v : vertices) {
        if (m_vertex_attribute[v.vid(*this)].on_surf &&
            !vertex_is_manifold(v)) { // vert is part of input and not manifold
            m_vertex_attribute[v.vid(*this)].label = 1;
        }
    }

    // count number nm vertices
    size_t nm_vertex_count = 0;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        if (m_vertex_attribute[v.vid(*this)].label == 1) {
            nm_vertex_count++;
        }
    }

    std::pair<size_t, size_t> ret_val(nm_edge_count, nm_vertex_count);
    return ret_val;
}


bool ManExtractMesh::edge_is_manifold(const Tuple& t) const
{
    if (!m_edge_attribute[t.eid(*this)].on_surf) { // only call for edges on input mesh
        return true;
    }

    // collect nb tets in input mesh
    auto nb_tets = get_incident_tets_for_edge(t);

    std::set<Tuple> nb_in_tets;
    for (const Tuple& nb_tet : nb_tets) {
        if (m_tet_attribute[nb_tet.tid(*this)].in_out) {
            // make nb_tet refer to edge of interest
            auto v_ids = oriented_tet_vids(nb_tet);
            std::vector<size_t> ordered_v_ids;
            ordered_v_ids.push_back(t.vid(*this));
            ordered_v_ids.push_back(t.switch_vertex(*this).vid(*this));
            for (size_t v_id : v_ids) {
                if (std::find(ordered_v_ids.begin(), ordered_v_ids.end(), v_id) ==
                    ordered_v_ids.end()) { // v_id not in ordered_v_ids
                    ordered_v_ids.push_back(v_id);
                }
            }
            Tuple new_tet = tuple_from_vids(
                ordered_v_ids[0],
                ordered_v_ids[1],
                ordered_v_ids[2],
                ordered_v_ids[3]);
            nb_in_tets.insert(new_tet);
        }
    }

    // run dfs
    std::set<size_t> visited_tids;
    edge_dfs_helper(visited_tids, *nb_in_tets.begin());

    // edge manifold iff all 'in' nb tets can be reached from every other
    return (visited_tids.size() == nb_in_tets.size());
}


void ManExtractMesh::edge_dfs_helper(std::set<size_t>& visited_tids, const Tuple& t) const
{
    // if tet isn't in input or already visited
    size_t curr_tid = t.tid(*this);
    if (!m_tet_attribute[curr_tid].in_out || visited_tids.count(curr_tid)) {
        return;
    }

    visited_tids.insert(curr_tid); // tet is in input mesh, and haven't visited before.

    auto t1 = t.switch_tetrahedron(*this);
    if (t1) {
        edge_dfs_helper(visited_tids, t1.value());
    }

    auto t2 = t.switch_face(*this).switch_tetrahedron(*this);
    if (t2) {
        edge_dfs_helper(visited_tids, t2.value());
    }
}


bool ManExtractMesh::vertex_is_manifold(const Tuple& t) const
{
    if (!m_vertex_attribute[t.vid(*this)]
             .on_surf) { // only call this function for vertices on input mesh
        return true;
    }

    auto nb_tets = get_one_ring_tets_for_vertex(t); // vector<Tuple>

    // find one ring tets in input mesh, make sure they refer to vertex of interest
    std::vector<Tuple> in_nb_tets;
    std::vector<Tuple> out_nb_tets;

    for (const Tuple& nb_tet : nb_tets) { // get vector of 'in' one ring tets of vertex
        auto v_ids = oriented_tet_vids(nb_tet);
        std::vector<size_t> ordered_vids;
        ordered_vids.push_back(t.vid(*this));
        for (size_t v_id : v_ids) {
            if (std::find(ordered_vids.begin(), ordered_vids.end(), v_id) ==
                ordered_vids.end()) { // v_id not in ordered_vids
                ordered_vids.push_back(v_id);
            }
        }
        Tuple new_tet =
            tuple_from_vids(ordered_vids[0], ordered_vids[1], ordered_vids[2], ordered_vids[3]);
        if (m_tet_attribute[nb_tet.tid(*this)].in_out) {
            in_nb_tets.push_back(new_tet);
        } else {
            out_nb_tets.push_back(new_tet);
        }
    }

    // if no 'out' tets, is manifold
    if (out_nb_tets.size() == 0) {
        return true;
    }

    std::vector<simplex::Face> out_b_faces; // won't be touched by 'in' call

    // run 'in' dfs from arbitrary in nb tet
    std::set<size_t> visited_tids_in;
    vertex_dfs_helper(visited_tids_in, in_nb_tets[0], true, out_b_faces);

    // run 'out' dfs from arbitrary out nb tet
    std::set<size_t> visited_tids_out;
    bool is_bound_v = is_boundary_vertex(t.vid(*this));
    if (is_bound_v) {
        out_b_faces.clear(); // should already be empty, just to be safe
        auto out_b_faces = get_boundary_faces_for_out_tets(t.vid(*this));
    } // otherwise leave empty
    vertex_dfs_helper(visited_tids_out, out_nb_tets[0], false, out_b_faces);

    bool res = (visited_tids_in.size() + visited_tids_out.size() == is_bound_v + nb_tets.size());
    if (!res && is_bound_v) {
        logger().warn("Vertex of offset input complex lies on mesh boundary: vid={}", t.vid(*this));
    }
    return res;
}


// external space is treated as a single 'out' polyhedra
void ManExtractMesh::vertex_dfs_helper(
    std::set<size_t>& visited_tids,
    const Tuple& t,
    const bool include,
    const std::vector<simplex::Face>& b_out_faces) const
{
    size_t curr_tid = t.tid(*this);
    if ((m_tet_attribute[curr_tid].in_out != include) ||
        visited_tids.count(curr_tid)) { // tet not in input, or has already been visited
        return;
    }

    visited_tids.insert(curr_tid); // first time visiting and is in input mesh, add to visited

    bool search_through_external_space =
        false; // whether to propagate search 'through' external 'out' space

    auto t1 = t.switch_tetrahedron(*this);
    if (t1) {
        vertex_dfs_helper(visited_tids, t1.value(), include, b_out_faces);
    } else if (!include) { // boundary face and we're searching through out tets
        search_through_external_space = true;
    }

    auto t2 = t.switch_face(*this).switch_tetrahedron(*this);
    if (t2) {
        vertex_dfs_helper(visited_tids, t2.value(), include, b_out_faces);
    } else if (!include) {
        search_through_external_space = true;
    }

    auto t3 = t.switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
    if (t3) {
        vertex_dfs_helper(visited_tids, t3.value(), include, b_out_faces);
    } else if (!include) {
        search_through_external_space = true;
    }

    if (search_through_external_space) { // search through all 'out' tets with boundary face
        visited_tids.insert(std::numeric_limits<size_t>::max()); // represents external space
        for (const simplex::Face& f : b_out_faces) {
            auto [ftup, _] = tuple_from_face(f.vertices()); // one possible tet
            vertex_dfs_helper(visited_tids, ftup, include, b_out_faces);
        }
    }
}


// NOTE: this function is very slow, but necessary to handle nonmanifold verts on boundary of mesh
bool ManExtractMesh::is_boundary_vertex(size_t vid) const
{
    auto t_ids = get_one_ring_tids_for_vertex(vid);
    for (size_t t_id : t_ids) {
        auto vs = oriented_tet_vids(t_id);
        for (int i = 0; i < 4; i++) {
            size_t v1 = vs[i];
            size_t v2 = vs[(i + 1) % 4];
            size_t v3 = vs[(i + 2) % 4];
            if ((v1 == vid) || (v2 == vid) || (v3 == vid)) {
                auto [ftup, _] = tuple_from_face({{v1, v2, v3}});
                if (!ftup.switch_tetrahedron(*this)) { // is boundary face
                    return true;
                }
            }
        }
    }
    return false;
}


std::vector<simplex::Face> ManExtractMesh::get_boundary_faces_for_out_tets(size_t vid) const
{
    std::vector<simplex::Face> b_out_faces;
    auto inc_tids = get_one_ring_tids_for_vertex(vid);
    for (const size_t& t_id : inc_tids) {
        if (m_tet_attribute[t_id].in_out) { // tet is 'in'
            continue;
        }
        auto vs = oriented_tet_vids(t_id);
        for (int i = 0; i < 4; i++) {
            size_t v1 = vs[i];
            size_t v2 = vs[(i + 1) % 4];
            size_t v3 = vs[(i + 2) % 4];
            auto [ftup, _] = tuple_from_face({{v1, v2, v3}});
            if (!ftup.switch_tetrahedron(*this)) { // boundary face
                b_out_faces.push_back(simplex::Face(v1, v2, v3));
            }
        }
    }
    return b_out_faces;
}


bool ManExtractMesh::is_simplicially_embedded() const
{
    int bad_tets = 0;
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        bad_tets += (!tet_is_simp_emb(t));
    }
    if (bad_tets == 0) {
        logger().info("\tMesh simplicially embedded: TRUE");
        return true;
    } else {
        logger().info("\tMesh simplicially embedded: FALSE ({} bad tets)", bad_tets);
        return false;
    }
}


bool ManExtractMesh::tet_is_simp_emb(const Tuple& t) const
{
    if (m_tet_attribute[t.tid(*this)].label == 1) { // tet is in input
        return true;
    }

    auto vs = oriented_tet_vids(t);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_attribute[vs[i]].label == 1) {
            vs_in.push_back(vs[i]);
        }
    }
    if (vs_in.size() <= 1) { // nothing or just one vertex in input
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t glob_eid = tuple_from_edge({{vs_in[0], vs_in[1]}}).eid(*this);
        return (m_edge_attribute[glob_eid].label == 1);
    } else if (vs_in.size() == 3) { // potentially one face in input
        auto [_, glob_fid] = tuple_from_face({{vs_in[0], vs_in[1], vs_in[2]}});
        return (m_face_attribute[glob_fid].label == 1);
    } else {
        return false;
    }
}


void ManExtractMesh::simplicial_embedding()
{
    // identify necessary tets to split (by vertices)
    std::vector<simplex::Tet> tets_to_split;
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);
        if (m_tet_attribute[tet.tid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 4; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 4];
                size_t v3 = vs[(i + 2) % 4];
                auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
                if (m_face_attribute[glob_fid].label != 1) {
                    to_split = false;
                }
            }

            if (to_split) { // tet not in input but all faces are
                tets_to_split.push_back(simplex::Tet(vs[0], vs[1], vs[2], vs[3]));
            }
        }
    }

    // actually split tets
    for (const simplex::Tet& tet : tets_to_split) {
        const auto& vs = tet.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2], vs[3]);
        std::vector<Tuple> garbage;
        if (!split_tet(t, garbage)) {
            log_and_throw_error("tet split failed! (simplicial embedding)");
        }
    }
    logger().info("\tTets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_attribute[f.fid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t glob_eid = tuple_from_edge({{v1, v2}}).eid(*this);
                if (m_edge_attribute[glob_eid].label != 1) {
                    to_split = false;
                }
            }

            if (to_split) {
                faces_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split faces
    for (const simplex::Face& f : faces_to_split) {
        const auto& vs = f.vertices();
        auto [t, _] = tuple_from_face({{vs[0], vs[1], vs[2]}});
        std::vector<Tuple> garbage;
        if (!split_face(t, garbage)) {
            log_and_throw_error("face split failed! (simplicial embedding)");
        }
    }
    logger().info("\tFaces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label != 1) { // edge not in input
            size_t v1_id = e.vid(*this);
            size_t v2_id = switch_vertex(e).vid(*this);
            if ((m_vertex_attribute[v1_id].label == 1) && (m_vertex_attribute[v2_id].label == 1)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        std::vector<Tuple> garbage;
        if (!split_edge(t, garbage)) {
            log_and_throw_error("edge split failed! (simplicial embedding)");
        }
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}


void ManExtractMesh::perform_offset()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        if ((m_vertex_attribute[v1].label + m_vertex_attribute[v2].label) == 1) {
            e_to_split.push_back(simplex::Edge(v1, v2));
        }
    }

    std::vector<Tuple> new_edges;
    for (const simplex::Edge& e : e_to_split) {
        // split edge
        new_edges.clear();
        Tuple t = tuple_from_edge(e.vertices());
        if (!split_edge(t, new_edges)) {
            log_and_throw_error("edge split failed! (marching tets)");
        }
    }

    // mark all offset tets (all tets with any vert labeled 1)
    // NOTE: I think this logic only works if the input complex is only edges and vertices
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);

        bool in_offset = false;
        for (size_t v : vs) { // vertices
            if (m_vertex_attribute[v].label == 1) {
                in_offset = true;
            }
        }

        if (in_offset) {
            m_tet_attribute[tet.tid(*this)].label = 2;
            if (m_params.manifold_union) {
                m_tet_attribute[tet.tid(*this)].in_out = true;
            } else {
                m_tet_attribute[tet.tid(*this)].in_out = false;
            }
        }
    }
}


void ManExtractMesh::set_offset_tags()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 2) {
            if (m_params.manifold_union) {
                m_tet_attribute[t_id].in_out = true;
                m_tet_attribute[t_id].tag = m_params.in_tag;
            } else {
                m_tet_attribute[t_id].in_out = false;
                m_tet_attribute[t_id].tag = m_params.replace_tag;
            }
        }
    }
}


void ManExtractMesh::extract_surface_mesh(MatrixXd& V, MatrixXi& F)
{
    // get vertices
    auto verts = get_vertices();
    V.resize(verts.size(), 3);
    for (const Tuple& v : verts) {
        size_t v_vid = v.vid(*this);
        Vector3d p = m_vertex_attribute[v_vid].m_posf;
        V.row(v_vid) = p;
    }

    // get faces (oriented properly)
    std::vector<std::array<size_t, 3>> faces_temp;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto other_tet = f.switch_tetrahedron(*this);
        if (other_tet) { // both tets exist
            Tuple in;
            if (!m_tet_attribute[f.tid(*this)].in_out &&
                m_tet_attribute[other_tet.value().tid(*this)].in_out) { // f tet is out, other is in
                in = other_tet.value();
            } else if (
                m_tet_attribute[f.tid(*this)].in_out &&
                !m_tet_attribute[other_tet.value().tid(*this)].in_out) {
                in = f;
            } else { // face is not boundary
                continue;
            }

            // face is on boundary
            size_t in_opp_v;
            auto face_vs = get_face_vids(in);
            auto tet_vs = oriented_tet_vids(in);
            for (size_t v : tet_vs) {
                if (std::find(face_vs.begin(), face_vs.end(), v) == face_vs.end()) {
                    in_opp_v = v;
                    break;
                }
            }

            // reorder verts if needed
            Vector3d p0 = m_vertex_attribute[face_vs[0]].m_posf;
            Vector3d p1 = m_vertex_attribute[face_vs[1]].m_posf;
            Vector3d p2 = m_vertex_attribute[face_vs[2]].m_posf;
            Vector3d m = (p0 + p1 + p2) / 3;
            Vector3d ni = m_vertex_attribute[in_opp_v].m_posf - m;
            Vector3d no = (p1 - p0).cross(p2 - p0);
            if (ni.dot(no) > 0) { // need to flip vert ordering
                size_t v_temp = face_vs[1];
                face_vs[1] = face_vs[2];
                face_vs[2] = v_temp;
            }

            // add face
            faces_temp.push_back(face_vs);
        } else { // face is on boundary
            if (m_tet_attribute[f.tid(*this)].in_out) {
                // identify opposite vert
                size_t v_other;
                auto face_vs = get_face_vids(f);
                auto tet_vs = oriented_tet_vids(f);
                for (const size_t v : tet_vs) {
                    if (std::find(face_vs.begin(), face_vs.end(), v) ==
                        face_vs.end()) { // v in tet_vs but not face_vs
                        v_other = v;
                        break;
                    }
                }

                // order face verts properly
                Vector3d p0 = m_vertex_attribute[face_vs[0]].m_posf;
                Vector3d p1 = m_vertex_attribute[face_vs[1]].m_posf;
                Vector3d p2 = m_vertex_attribute[face_vs[2]].m_posf;
                Vector3d m = (p0 + p1 + p2) / 3;
                Vector3d nf = (p1 - p0).cross(p2 - p0);
                Vector3d ni = m_vertex_attribute[v_other].m_posf - m;
                if (ni.dot(nf) > 0) {
                    size_t temp_vid = face_vs[1];
                    face_vs[1] = face_vs[2];
                    face_vs[2] = temp_vid;
                }

                // add face
                faces_temp.push_back(face_vs);
            }
        }
    }

    // store in F matrix
    F.resize(faces_temp.size(), 3);
    for (int i = 0; i < F.rows(); i++) {
        F.row(i) << faces_temp[i][0], faces_temp[i][1], faces_temp[i][2];
    }
}


bool ManExtractMesh::invariants(const std::vector<Tuple>& tets)
{
    igl::predicates::exactinit();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);

        if (res != igl::predicates::Orientation::NEGATIVE) {
            return false;
        }
    }
    return true;
}


void ManExtractMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(vertex_size(),
                             -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<std::vector<int>> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset; // vertices to offset
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].label == 1) {
            verts_to_offset.push_back(m_vertex_attribute[i].m_posf);
            vid_map[i] = verts_to_offset.size() - 1;
        }
    }
    Eigen::MatrixXd V(verts_to_offset.size(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = verts_to_offset[i];
    }

    // get all offset input edges
    auto edges = get_edges();
    for (const Tuple e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            std::vector<int> curr_e;
            curr_e.push_back(vid_map[e.vid(*this)]);
            curr_e.push_back(vid_map[switch_vertex(e).vid(*this)]);
            cells.push_back(curr_e);
        }
    }

    // get all offset input faces
    auto faces = get_faces();
    for (const Tuple f : faces) {
        if (m_face_attribute[f.fid(*this)].label == 1) {
            std::vector<int> curr_f;
            curr_f.push_back(vid_map[f.vid(*this)]);
            Tuple f1 = switch_vertex(f);
            curr_f.push_back(vid_map[f1.vid(*this)]);
            Tuple f2 = switch_vertex(switch_edge(f1));
            curr_f.push_back(vid_map[f2.vid(*this)]);
            cells.push_back(curr_f);
        }
    }

    // get all offset input tets ( this should never happen? )
    bool flag = false;
    int num_tets = tet_size();
    for (size_t i = 0; i < num_tets; i++) {
        if (m_tet_attribute[i].label == 1) {
            flag = true;
            auto vids = oriented_tet_vids(i);
            std::vector<int> curr_t;
            for (const size_t vid : vids) {
                curr_t.push_back(vid_map[vid]);
            }
        }
    }
    if (flag) {
        logger().warn("One or more tet included in complex to offset.");
    };

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void ManExtractMesh::write_surface(const std::string& path)
{
    logger().info("Write surface {}.obj", path);
    MatrixXd pre_V_out;
    MatrixXi pre_F_out;
    extract_surface_mesh(pre_V_out, pre_F_out);
    MatrixXd pre_V_out_reduced;
    MatrixXi pre_F_out_reduced;
    MatrixXi pre_I; // index map, don't actually need
    MatrixXi pre_B; // dummy variable
    igl::remove_unreferenced(pre_V_out, pre_F_out, pre_V_out_reduced, pre_F_out_reduced, pre_I);
    logger().info("\tEdge manifoldness check: {}", igl::is_edge_manifold(pre_F_out_reduced));
    logger().info(
        "\tVertex manifoldness check: {}",
        igl::is_vertex_manifold(pre_F_out_reduced, pre_B));
    igl::write_triangle_mesh(path + ".obj", pre_V_out_reduced, pre_F_out_reduced);
}


void ManExtractMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu", path);
    logger().warn("Any new tags added during manifold extraction will not appear in vtu.");

    consolidate_mesh();
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);

    V.setZero();
    T.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));

    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);

        // set tet tags
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](t_id, 0) = (m_tet_attribute[t_id].tag.count(i) == 1) ? 1 : 0;
        }

        // set tet verts
        const auto& loc_vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(t.tid(*this), j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    for (int i = 0; i < m_tags_count; i++) {
        writer->add_cell_field(fmt::format("tag_{}", i), tags[i]);
    }
    writer->write_mesh(path + ".vtu", V, T);

    // envelope
    if (m_has_envelope) {
        const auto out_surf_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        logger().info("Write {}", out_surf_path);
        surf_writer->write_mesh(out_surf_path, m_V_envelope, m_F_envelope);
    }
}


void ManExtractMesh::write_msh_groups(const std::string& path)
{
    logger().info("Write {}_groups.msh", path);
    consolidate_mesh();

    wmtk::MshData msh;

    // set vertices
    const auto& verts = get_vertices();
    msh.add_tet_vertices(verts.size(), [&](size_t k) {
        auto i = verts[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();

    int64_t max_tag = -1;
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        const auto& tag = m_tet_attribute[t_id].tag;
        if (tag.empty()) {
            continue;
        }
        int64_t mt = *tag.rbegin();
        max_tag = std::max(max_tag, mt);
    }

    if (m_tags_count < max_tag + 1) {
        logger().warn(
            "Max tag is {} but but m_tags_count is {}. Adjusting m_tags_count",
            max_tag,
            m_tags_count);
        m_tags_count = max_tag + 1;
    }

    std::vector<Tuple> tets_with_tag;
    tets_with_tag.reserve(tets.size());

    auto msh_add_tets = [&]() {
        msh.add_tets(tets_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tet_vids(tets_with_tag[k]);
            std::array<size_t, 4> data;
            for (int j = 0; j < 4; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // add ambient mesh
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].tag.empty()) {
            tets_with_tag.push_back(t);
        }
    }
    msh_add_tets();
    msh.add_physical_group("ambient");

    // group for each tag
    for (size_t tag_img = 0; tag_img < m_tags_count; tag_img++) {
        tets_with_tag.clear();
        for (const Tuple& t : tets) {
            size_t t_id = t.tid(*this);
            if (m_tet_attribute[t_id].tag.count(tag_img) != 0) {
                tets_with_tag.push_back(t);
            }
        }

        if (tets_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(3);
        msh_add_tets();

        const std::string group_name = fmt::format("tag_{}", tag_img);
        msh.add_physical_group(group_name);
    }

    if (m_has_envelope) {
        msh.add_edge_vertices(m_V_envelope.rows(), [this](size_t k) {
            return m_V_envelope.row(k);
        });
        msh.add_edges(m_F_envelope.rows(), [this](size_t k) { return m_F_envelope.row(k); });
        msh.add_physical_group("EnvelopeSurface");
    }

    msh.save(path + "_groups.msh", true);
}


} // namespace wmtk::components::manifold_extraction