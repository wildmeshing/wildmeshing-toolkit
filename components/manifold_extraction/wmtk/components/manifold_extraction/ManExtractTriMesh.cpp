#include "ManExtractTriMesh.h"
#include <wmtk/utils/Logger.hpp>

// clang-format off
#include <igl/adjacency_list.h>
#include <igl/remove_unreferenced.h>
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <fstream>
#include <iomanip>
#include <set>


namespace wmtk::components::manifold_extraction {


bool ManExtractTriMesh::is_curve_vertex(const Tuple& v) const
{
    const size_t v_id = v.vid(*this);
    auto inc_edges = get_one_ring_edges_for_vertex(v_id);
    for (const Tuple& e : inc_edges) {
        if (is_curve_edge(e)) {
            return true;
        }
    }

    return false; // no incident edges are on curve
}


bool ManExtractTriMesh::is_curve_edge(const Tuple& e) const
{
    auto other = e.switch_face(*this);
    if (other) {
        return (is_interior_face(other.value()) != is_interior_face(e));
    } else {
        return is_interior_face(e);
    }
}


bool ManExtractTriMesh::is_interior_face(const Tuple& f) const
{
    return is_interior_face(f.fid(*this));
}
bool ManExtractTriMesh::is_interior_face(size_t f_id) const
{
    return m_man_params.tag_selection->eval(m_face_attribute[f_id].tag);
}


size_t ManExtractTriMesh::label_non_manifold()
{
    // label nm vertices
    auto vertices = get_vertices();
    size_t ret_num = 0;
    for (const Tuple& v : vertices) {
        if (is_curve_vertex(v) &&
            !vertex_is_manifold(v)) { // vert is part of input and not manifold
            m_vertex_attribute[v.vid(*this)].label = 1;
            ret_num++;
        }
    }
    return ret_num;
}


bool ManExtractTriMesh::vertex_is_manifold(const Tuple& v) const
{
    if (!is_curve_vertex(v)) { // only call this function for vertices on input mesh
        return true;
    }

    const size_t v0_id = v.vid(*this);
    auto nb_faces = get_one_ring_tris_for_vertex(v);

    // collect nb faces inside curve
    std::set<Tuple> nb_in_faces;
    for (const Tuple& nb_face : nb_faces) {
        if (is_interior_face(nb_face)) {
            // make tuple point to vertex
            auto vs = oriented_tri_vids(nb_face);
            std::vector<size_t> ordered_vids(1, v0_id);
            for (const size_t& v_id : vs) {
                if (v_id != v0_id) {
                    ordered_vids.push_back(v_id);
                }
            }
            Tuple newtup = tuple_from_vids(ordered_vids[0], ordered_vids[1], ordered_vids[2]);
            nb_in_faces.insert(newtup);
        }
    }

    // run dfs
    std::set<size_t> visited_fids;
    vertex_dfs_helper(visited_fids, *nb_in_faces.begin());
    return (visited_fids.size() == nb_in_faces.size());
}


void ManExtractTriMesh::vertex_dfs_helper(std::set<size_t>& visited_fids, const Tuple& f) const
{
    const size_t curr_fid = f.fid(*this);

    // face is not interior or is already visited
    if ((!is_interior_face(curr_fid)) || visited_fids.count(curr_fid)) {
        return;
    }

    visited_fids.insert(curr_fid);
    auto f1 = f.switch_face(*this);
    if (f1) {
        vertex_dfs_helper(visited_fids, f1.value());
    }
    auto f2 = f.switch_edge(*this).switch_face(*this);
    if (f2) {
        vertex_dfs_helper(visited_fids, f2.value());
    }
}


void ManExtractTriMesh::extract_curve_mesh(MatrixXd& V, MatrixXi& F) const
{
    // create V matrix
    const auto& verts = get_vertices();
    V.resize(verts.size(), 3);
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        Vector2d p2d = m_vertex_attribute[v_id].m_posf;
        Vector3d p(p2d(0), p2d(1), 0);
        V.row(v_id) = p;
    }

    // collect curve edges
    std::vector<std::array<size_t, 2>> curve_edges;
    const auto& edges = get_edges();
    for (const Tuple& e : edges) {
        if (is_curve_edge(e)) {
            curve_edges.push_back({e.vid(*this), e.switch_vertex(*this).vid(*this)});
        }
    }

    // create F matrix
    F.resize(curve_edges.size(), 2);
    for (size_t i = 0; i < curve_edges.size(); i++) {
        F.row(i) << curve_edges[i][0], curve_edges[i][1];
    }
}


void ManExtractTriMesh::write_curve(const std::string& path)
{
    logger().info("Write curve {}.obj", path);
    MatrixXd pre_V_out;
    MatrixXi pre_F_out;
    extract_curve_mesh(pre_V_out, pre_F_out);
    MatrixXd V_out;
    MatrixXi F_out;
    MatrixXi pre_I; // index map, don't actually need
    igl::remove_unreferenced(pre_V_out, pre_F_out, V_out, F_out, pre_I);

    // compute adjacency matrix to get adjacent vertices
    std::vector<std::vector<int>> adj;
    igl::adjacency_list(F_out, adj);

    // manifold check, every vertex has 2 neighbors
    bool is_manifold = true;
    for (const auto& nbrs : adj) {
        if (nbrs.size() != 2) {
            is_manifold = false;
            break;
        }
    }
    if (is_manifold) {
        logger().info("\tCurve manifoldness check passed.");
    } else {
        logger().warn(
            "\tCurve manifoldness check failed. Likely due to a nonmanifold vertex on the mesh "
            "boundary.");
    }


    // orient edges to form consistently ordered loop(s)
    std::vector<std::array<int, 2>> ordered_edges;
    if (is_manifold) {
        std::vector<bool> visited(adj.size(), false);
        for (int start = 0; start < static_cast<int>(adj.size()); ++start) {
            if (visited[start]) continue;

            std::vector<int> loop;
            int prev = -1;
            int curr = start;
            do {
                loop.push_back(curr);
                visited[curr] = true;
                int next = (adj[curr][0] != prev) ? adj[curr][0] : adj[curr][1];
                prev = curr;
                curr = next;
            } while (curr != start);

            for (size_t i = 0; i < loop.size(); i++) {
                ordered_edges.push_back({loop[i], loop[(i + 1) % loop.size()]});
            }
        }
    } else { // should never happen
        for (int i = 0; i < F_out.rows(); i++) {
            ordered_edges.push_back({F_out(i, 0), F_out(i, 1)});
        }
    }

    // write obj file (v <x> <y> <z> ... l <v1> <v2> ...)
    std::ofstream out(path + ".obj");
    out << std::setprecision(17);
    for (int i = 0; i < V_out.rows(); i++) {
        out << "v " << V_out(i, 0) << " " << V_out(i, 1) << " " << V_out(i, 2) << "\n";
    }
    for (const auto& e : ordered_edges) {
        out << "l " << (e[0] + 1) << " " << (e[1] + 1) << "\n"; // OBJ is 1-indexed
    }
}


} // namespace wmtk::components::manifold_extraction