#include "ManExtractTetMesh.h"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldCheck.hpp>

// clang-format off
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

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
            m_vertex_extra[e.vid(*this)].label = 1;
            m_vertex_extra[e.switch_vertex(*this).vid(*this)].label = 1;
            nm_edge_count++;
        }
    }

    // label nm vertices
    auto vertices = get_vertices();
    for (const Tuple& v : vertices) {
        if (is_surface_vertex(v) &&
            !vertex_is_manifold(v)) { // vert is part of input and not manifold
            m_vertex_extra[v.vid(*this)].label = 1;
        }
    }

    // count number nm vertices
    size_t nm_vertex_count = 0;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        if (m_vertex_extra[v.vid(*this)].label == 1) {
            nm_vertex_count++;
        }
    }

    std::pair<size_t, size_t> ret_val(nm_edge_count, nm_vertex_count);
    return ret_val;
}


bool ManExtractTetMesh::edge_is_manifold(const Tuple& t) const
{
    return wmtk::utils::edge_is_manifold(*this, t, [this](const size_t tid) {
        return is_interior_tet(tid);
    });
}


bool ManExtractTetMesh::vertex_is_manifold(const Tuple& t) const
{
    const bool res = wmtk::utils::vertex_is_manifold(*this, t, [this](const size_t tid) {
        return is_interior_tet(tid);
    });
    if (!res && is_boundary_vertex(t)) {
        logger().warn("Vertex of offset input complex lies on mesh boundary: vid={}", t.vid(*this));
    }
    return res;
}


// NOTE: this is really slow. there is probably a better way of doing things
bool ManExtractTetMesh::is_boundary_vertex(const Tuple& v) const
{
    const size_t v_id = v.vid(*this);
    auto t_ids = get_one_ring_tids_for_vertex(v_id);
    for (size_t t_id : t_ids) {
        auto vs = oriented_tet_vids(t_id);
        for (int i = 0; i < 4; i++) {
            size_t v1 = vs[i];
            size_t v2 = vs[(i + 1) % 4];
            size_t v3 = vs[(i + 2) % 4];
            if ((v1 == v_id) || (v2 == v_id) || (v3 == v_id)) {
                auto [ftup, _] = tuple_from_face({{v1, v2, v3}});
                if (!ftup.switch_tetrahedron(*this)) { // is boundary face
                    return true;
                }
            }
        }
    }
    return false;
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
    if (!m_params.output_enabled) return;
    logger().info("Write surface {}.obj", path);
    MatrixXd pre_V_out;
    MatrixXi pre_F_out;
    extract_surface_mesh(pre_V_out, pre_F_out);
    MatrixXd V_out;
    MatrixXi F_out;
    MatrixXi pre_I; // index map, don't actually need
    MatrixXi pre_B; // dummy variable
    igl::remove_unreferenced(pre_V_out, pre_F_out, V_out, F_out, pre_I);

    bool vertex_manifold = igl::is_vertex_manifold(F_out, pre_B);
    if (vertex_manifold) {
        logger().info("\tVertex manifoldness check passed.");
    } else {
        logger().warn(
            "\tSurface is not vertex manifold. Likely due to a nonmanifold vertex/edge on the mesh "
            "boundary.");
    }

    bool edge_manifold = igl::is_edge_manifold(F_out);
    if (edge_manifold) {
        logger().info("\tEdge manifoldness check passed.");
    } else {
        logger().warn(
            "\tSurface is not edge manifold. Likely due to a nonmanifold vertex/edge on the mesh "
            "boundary.");
    }

    igl::write_triangle_mesh(path + ".obj", V_out, F_out);
}


} // namespace wmtk::components::manifold_extraction