#include "topology_separate_2d.hpp"
#include <iostream>
namespace wmtk::components::internal {

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m)
{
    /*
    Algorithm idea:
    first partition the whole mesh to be m edge-connected components
    for each vertex as the connecting vertex of multiple(n) edge-connected components,
    for edge-cc i in n, if this vertex is shared by a_i groups of
    vertex-connected but not edge-connected triangles, then create a_i copies of this vertex
    assign one to each group. So a total of \Sigma_{i = 1}^{n} a_i copies of the same vertex will be
    created

    Algorithm steps: (for 2d)
    1. find all the ecc relationships in this trimesh
    2. for each "real" vertex,
            init the default nb_cp = 1 to this vertex
            if found 2 different triangles vertex-connected at this vertex but not edge-connected,
            then assign nb_cp = \Sigma_{i = 1}^{k} a_i  , where k is the number of ecc adj to this
    vertex, a_i is the number of groups of vertex-connected but not edge-connected triangles in the
    i-th ecc
    3. init a counter starting from 0. For vertex starting from 0, for each "real" vertex:
            if only 1 copy, assign vertex with a single current counter value, then increment
            else assign vertex with {curr, ..., ctr + nb_cp - 1}, then increment by num of copies
    4. use the new ids to reconstruct the mesh
    5. return the new mesh
    */
    std::vector<wmtk::Tuple> faces = m.get_all(wmtk::PrimitiveType::Face);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    int nb_vertex = vertices.size();
    int nb_tri = faces.size();
    std::vector<long> vertex_cp(nb_vertex, 1); // how many copies should we make on this vertex

    // std::cout << "# of tris = " << nb_tri << std::endl;
    // std::cout << "# of vertices = " << nb_vertex << std::endl;

    // Prior work: build an face-adjacency list of triangle faces
    std::vector<std::vector<long>> adj_list_faces(nb_tri, std::vector<long>());
    for (long i = 0; i < nb_tri; ++i) {
        for (long j = i; j < nb_tri; ++j) {
            long edge_con =
                edge_connected(m, wmtk::Simplex::face(faces[i]), wmtk::Simplex::face(faces[j]));
            if (edge_con != -1) {
                if (i != j) {
                    adj_list_faces[i].push_back(j);
                    adj_list_faces[j].push_back(i);
                } else {
                    adj_list_faces[i].push_back(j);
                }
            }
        }
    }
    std::cout << "adj_list_tets = " << std::endl;
    for (int i = 0; i < nb_tri; ++i) {
        std::cout << i << ": ";
        for (auto j : adj_list_faces[i]) std::cout << j << " ";
        std::cout << std::endl;
    }

    // Step 1: constuct a list of edge-connected components
    std::vector<std::vector<long>> face_cc_list;
    std::vector<bool> visited_faces(nb_tri, false);
    auto condition = [](long face, std::vector<long>&) noexcept { return true; };
    std::vector<long> nullvector = std::vector<long>(nb_tri);
    std::iota(nullvector.begin(), nullvector.end(), 1);
    for (long i = 0; i < nb_tri; ++i) {
        if (visited_faces[i] == false) {
            std::vector<long> cc;
            dfs(i, visited_faces, cc, adj_list_faces, condition, nullvector);
            face_cc_list.push_back(cc);
        }
    }
    long nb_cc = face_cc_list.size();
    // std::cout << "# of cc = " << nb_cc << std::endl;
    // print_vv(face_cc_list);
    std::vector<long> nb_cc_vec(nb_cc);
    for (long i = 0; i < nb_cc; ++i) nb_cc_vec[i] = face_cc_list[i].size();

    // Step 2: for each vertex on the boundary, count number of group tris around it
    // start a version of the algo where we loop over vertices instead of triangles
    std::map<long, std::vector<std::vector<long>>> ccav_vector;
    for (long i = 0; i < nb_vertex; ++i) {
        if (m.is_boundary(vertices[i], PrimitiveType::Vertex)) {
            // std::cout << "vertex " << i << " is on boundary. The adjacent faces are: ";
            std::vector<long> adj_faces = adj_faces_of_vertex(m, i);
            // for (auto j : adj_faces) std::cout << j << " ";
            // std::cout << std::endl;
            std::vector<std::vector<long>> ccav = cc_around_vertex(m, adj_faces, adj_list_faces);
            vertex_cp[i] = ccav.size();
            ccav_vector[i] = ccav;
        }
    }
    // for (auto j : vertex_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 3: assign queues to each vertex
    int counter = 0;
    std::vector<std::vector<long>> new_id_of_vertex(nb_vertex, std::vector<long>());
    for (int i = 0; i < nb_vertex; ++i) {
        for (int j = 0; j < vertex_cp[i]; ++j) {
            new_id_of_vertex[i].push_back(counter);
            counter++;
        }
    }

    // Step 4: reconstruct the mesh
    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri, 3);
    for (long i = 0; i < nb_tri; ++i) {
        std::vector<wmtk::Tuple> list = wmtk::simplex::faces_single_dimension(
            m,
            wmtk::Simplex::face(faces[i]),
            PrimitiveType::Vertex);
        std::vector<long> data(3, -1);
        for (int index = 0; index < 3; ++index) {
            long id_v = find_vertex_index(m, list[index]);
            if (vertex_cp[id_v] == 1)
                data[index] = new_id_of_vertex[id_v][0];
            else {
                for (long j = 0; j < ccav_vector[id_v].size(); ++j) {
                    if (std::find(ccav_vector[id_v][j].begin(), ccav_vector[id_v][j].end(), i) !=
                        ccav_vector[id_v][j].end()) {
                        data[index] = new_id_of_vertex[id_v][j];
                        break;
                    }
                }
            }
        }
        // std::cout << "data = " << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
        tris.row(i) << data[0], data[1], data[2];
    }
    mesh.initialize(tris); // init the topology

    return mesh;
}
} // namespace wmtk::components::internal
