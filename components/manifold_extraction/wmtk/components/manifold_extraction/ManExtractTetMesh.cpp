#include "ManExtractTetMesh.h"
#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include "wmtk/utils/Rational.hpp"

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

#include <cmath>
#include <limits>
#include <paraviewo/VTUWriter.hpp>
#include <set>


namespace wmtk::components::manifold_extraction {


bool ManExtractTetMesh::is_surface_vertex(const Tuple& v) const
{
    size_t v0_id = v.vid(*this);
    auto inc_tids = get_one_ring_tids_for_vertex(v.vid(*this));
    std::set<simplex::Face> inc_faces;
    for (const size_t& t_id : inc_tids) {
        std::vector<size_t> tet_oppo_vids;
        for (const size_t& v_id : oriented_tet_vids(t_id)) {
            if (v_id != v0_id) {
                tet_oppo_vids.push_back(v_id);
            }
        }
        for (size_t i = 0; i < 3; i++) {
            inc_faces.insert(simplex::Face(v0_id, tet_oppo_vids[i], tet_oppo_vids[(i + 1) % 3]));
        }
    }

    for (const simplex::Face f : inc_faces) {
        auto [f_tup, _] = tuple_from_face(f);
        if (is_surface_face(f_tup)) {
            return true;
        }
    }
    return false; // no incident faces are surface faces
}


bool ManExtractTetMesh::is_surface_edge(const Tuple& e) const
{
    const size_t v1 = e.vid(*this);
    const size_t v2 = e.switch_vertex(*this).vid(*this);
    const simplex::Edge edge(v1, v2);
    std::set<size_t> link_vids;
    for (const Tuple& t : get_incident_tets_for_edge(e)) {
        const simplex::Tet tet = simplex_from_tet(t.tid(*this));
        const simplex::Edge e_opp = tet.opposite_edge(edge);
        link_vids.insert(e_opp.vertices()[0]);
        link_vids.insert(e_opp.vertices()[1]);
    }

    for (const size_t& v_id : link_vids) {
        auto [f_tup, _] = tuple_from_face({{v_id, v1, v2}});
        if (is_surface_face(f_tup)) {
            return true;
        }
    }
    return false; // no incident faces are on surface
}


bool ManExtractTetMesh::is_surface_face(const Tuple& f) const
{
    auto other = f.switch_tetrahedron(*this);
    if (other) {
        return (is_interior_tet(other.value()) != is_interior_tet(f));
    } else {
        return is_interior_tet(f);
    }
}


bool ManExtractTetMesh::is_interior_tet(const Tuple& t) const
{
    return is_interior_tet(t.tid(*this));
}
bool ManExtractTetMesh::is_interior_tet(const size_t& t_id) const
{
    return m_man_params.tag_selection->eval(m_tet_attribute[t_id].tag);
}


std::pair<size_t, size_t> ManExtractTetMesh::label_non_manifold()
{
    // label nm edges
    size_t nm_edge_count = 0;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (is_surface_edge(e) &&
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
        if (is_surface_vertex(v) &&
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


bool ManExtractTetMesh::edge_is_manifold(const Tuple& t) const
{
    if (!is_surface_edge(t)) { // only call for edges on input mesh
        return true;
    }

    // collect nb tets in input mesh
    auto nb_tets = get_incident_tets_for_edge(t);

    std::set<Tuple> nb_in_tets;
    for (const Tuple& nb_tet : nb_tets) {
        if (is_interior_tet(nb_tet)) {
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


void ManExtractTetMesh::edge_dfs_helper(std::set<size_t>& visited_tids, const Tuple& t) const
{
    // if tet isn't in input or already visited
    size_t curr_tid = t.tid(*this);
    if (!is_interior_tet(curr_tid) || visited_tids.count(curr_tid)) {
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


bool ManExtractTetMesh::vertex_is_manifold(const Tuple& t) const
{
    if (!is_surface_vertex(t)) { // only call this function for vertices on input mesh
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
        if (is_interior_tet(nb_tet)) {
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
void ManExtractTetMesh::vertex_dfs_helper(
    std::set<size_t>& visited_tids,
    const Tuple& t,
    const bool include,
    const std::vector<simplex::Face>& b_out_faces) const
{
    size_t curr_tid = t.tid(*this);
    if ((is_interior_tet(curr_tid) != include) ||
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


// NOTE: this function is very slow, but necessary to handle nonmanifold verts on boundary of
// mesh
bool ManExtractTetMesh::is_boundary_vertex(size_t vid) const
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


std::vector<simplex::Face> ManExtractTetMesh::get_boundary_faces_for_out_tets(size_t vid) const
{
    std::vector<simplex::Face> b_out_faces;
    auto inc_tids = get_one_ring_tids_for_vertex(vid);
    for (const size_t& t_id : inc_tids) {
        if (is_interior_tet(t_id)) { // tet is 'in'
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


void ManExtractTetMesh::extract_surface_mesh(MatrixXd& V, MatrixXi& F)
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
            if (!is_interior_tet(f) &&
                is_interior_tet(other_tet.value())) { // f tet is out, other is in
                in = other_tet.value();
            } else if (is_interior_tet(f) && !is_interior_tet(other_tet.value())) {
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
            if (is_interior_tet(f)) {
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


void ManExtractTetMesh::write_surface(const std::string& path)
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


} // namespace wmtk::components::manifold_extraction