#include "topology_separate_3d.hpp"

namespace wmtk::components::internal {

wmtk::TetMesh topology_separate_3d_old(wmtk::TetMesh m)
{
    std::vector<wmtk::Tuple> edges = m.get_all(wmtk::PrimitiveType::Edge);
    std::vector<wmtk::Tuple> tets = m.get_all(wmtk::PrimitiveType::Tetrahedron);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    long nb_vertex = vertices.size();
    long nb_tet = tets.size();
    long nb_edge = edges.size();

    std::vector<long> edge_cp(nb_edge, 1); // how many copies should we make on this edge
    std::vector<long> vertex_cp(nb_vertex, 0); // how many copies should we make on this vertex

    // std::cout << "\n# of tets = " << nb_tet << std::endl;
    // std::cout << "# of edges = " << nb_edge << std::endl;
    // std::cout << "# of vertices = " << nb_vertex << std::endl;

    // for (int i = 0; i < nb_tet; ++i) {
    //     std::cout << "tet " << i << ": \n";
    //     auto ee = wmtk::simplex::faces_single_dimension(
    //         m,
    //         wmtk::Simplex::tetrahedron(tets[i]),
    //         PrimitiveType::Edge);
    //     for (int j = 0; j < 6; ++j) {
    //         auto vers = wmtk::simplex::faces_single_dimension(m, wmtk::Simplex::edge(ee[j]),
    //         PrimitiveType::Vertex); std::cout << "edge " << find_vertex_index(m, vers[0]) <<
    //         find_vertex_index(m, vers[1]) << "with index " << find_edge_index(m, ee[j]) <<
    //         std::endl;
    //     }
    // }

    // Prior work: build an face-adjacency list of tets
    std::vector<std::vector<long>> adj_list_tets(nb_tet, std::vector<long>());
    for (long i = 0; i < nb_tet; ++i) {
        for (long j = i; j < nb_tet; ++j) {
            long face_con = face_connected(
                m,
                wmtk::Simplex::tetrahedron(tets[i]),
                wmtk::Simplex::tetrahedron(tets[j]));
            if (face_con != -1) {
                if (i != j) {
                    adj_list_tets[i].push_back(j);
                    adj_list_tets[j].push_back(i);
                } else {
                    adj_list_tets[i].push_back(j);
                }
            }
        }
    }
    // std::cout << "adj_list_tets = " << std::endl;
    // for (int i = 0; i < nb_tet; ++i) {
    //     std::cout << i << ": ";
    //     for (auto j : adj_list_tets[i]) std::cout << j << " ";
    //     std::cout << std::endl;
    // }

    // Step 1: constuct a list of face-connected components
    std::vector<std::vector<long>> face_cc_list;
    std::vector<bool> visited_faces(nb_tet, false);
    auto condition = [](long face, std::vector<long>&) noexcept { return true; };
    std::vector<long> nullvector = std::vector<long>(nb_tet);
    std::iota(nullvector.begin(), nullvector.end(), 1);
    for (long i = 0; i < nb_tet; ++i) {
        if (visited_faces[i] == false) {
            std::vector<long> cc;
            dfs(i, visited_faces, cc, adj_list_tets, condition, nullvector);
            face_cc_list.push_back(cc);
        }
    }
    long nb_cc = face_cc_list.size();
    // std::cout << "# of cc = " << nb_cc << std::endl;
    std::vector<long> nb_cc_vec(nb_cc);
    for (long i = 0; i < nb_cc; ++i) nb_cc_vec[i] = face_cc_list[i].size();

    // Step 1.5: construct the inverse, tet -> cc
    std::vector<long> tet_cc(nb_tet);
    for (long i = 0; i < face_cc_list.size(); ++i) {
        for (long j : face_cc_list[i]) tet_cc[j] = i;
    }

    // Step 2: for each edge on the boundary, count number of group tets around it
    // std::map<long, std::vector<std::vector<long>>> ccav_vector;
    std::map<long, std::vector<long>> ccav_vector;
    for (long i = 0; i < nb_edge; ++i) {
        if (m.is_boundary(edges[i], PrimitiveType::Edge)) {
            // std::cout << "edge " << i << " is on boundary. ";
            // auto vers = wmtk::simplex::faces_single_dimension(m, wmtk::Simplex::edge(edges[i]),
            // PrimitiveType::Vertex); std::cout << " edge " << find_vertex_index(m, vers[0]) <<
            // find_vertex_index(m, vers[1]) << std::endl;
            // std::cout << "The adjacent faces are: ";
            std::vector<long> adj_tets = adj_tets_of_edge(m, i);
            // for (auto j : adj_tets) std::cout << j << " ";
            // std::cout << std::endl;
            std::vector<std::vector<long>> ccav = tet_cc_around_tuple(m, adj_tets, adj_list_tets);
            edge_cp[i] = ccav.size();
            std::vector<long> cc;
            for (std::vector<long> j : ccav) cc.push_back(tet_cc[j[0]]);
            ccav_vector[i] = cc;
            // std::cout << "ccav.size() = " << ccav.size() << std::endl;
            // ccav_vector[i] = ccav;
        }
    }
    // for (long j : edge_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 2.5(with question): load the number of copies of edges to vertices
    // CAREFUL: copies of a vertex should be the number of face-connected components in tets around
    // all non-manifold edges incident at this vertex
    std::vector<std::set<long>> vertex_cc_set(nb_vertex, std::set<long>());
    for (int i = 0; i < nb_edge; ++i) {
        if (edge_cp[i] != 1) {
            std::vector<wmtk::Tuple> edge_vertices = wmtk::simplex::faces_single_dimension(
                m,
                wmtk::Simplex::edge(edges[i]),
                PrimitiveType::Vertex);
            vertex_cc_set[find_vertex_index(m, edge_vertices[0])].insert(
                ccav_vector[i].begin(),
                ccav_vector[i].end());
            vertex_cc_set[find_vertex_index(m, edge_vertices[1])].insert(
                ccav_vector[i].begin(),
                ccav_vector[i].end());
            // vertex_cp[find_vertex_index(m, edge_vertices[0])] += edge_cp[i];
            // vertex_cp[find_vertex_index(m, edge_vertices[1])] += edge_cp[i];
        }
    }
    for (int i = 0; i < nb_vertex; ++i) vertex_cp[i] += vertex_cc_set[i].size();
    // for (long j : vertex_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 2.9: deal with vertices on the boundary, count number of group tets around it. Make sure
    // the already edge-connected ones are not in count
    std::map<long, std::vector<std::vector<long>>> ccav_vector_vertex;
    for (long i = 0; i < nb_vertex; ++i) {
        if (m.is_boundary(vertices[i], PrimitiveType::Vertex)) {
            // std::cout << "vertex " << i << " is on boundary. The adjacent faces are: ";
            std::vector<long> adj_tets = adj_tets_of_vertex(m, i);
            // for (long j : adj_tets) std::cout << j << " ";
            // std::cout << std::endl;
            std::vector<std::vector<long>> ccav = tet_cc_around_tuple(m, adj_tets, adj_list_tets);
            // std::cout << "ccav.size() = " << ccav.size() << std::endl;
            // for more than 1 cc, we need to check if 2 cc are connected by edges, which we have
            // already counted
            if (ccav.size() > 1) {
                std::vector<bool> cc_flag(ccav.size(), true);
                // std::cout << "before processing, cc_flag = ";
                // for (bool f : cc_flag) std::cout << f << " ";
                // std::cout << std::endl;
                for (int j = 0; j < ccav.size(); ++j) {
                    for (int k = j + 1; k < ccav.size(); ++k) {
                        if (cc_flag[j] == false && cc_flag[k] == false) continue;
                        if (edge_connected(
                                m,
                                wmtk::Simplex::tetrahedron(tets[ccav[j][0]]),
                                wmtk::Simplex::tetrahedron(tets[ccav[k][0]]))) {
                            cc_flag[j] = false;
                            cc_flag[k] = false;
                            break;
                        }
                    }
                }
                // std::cout << "after processing, cc_flag = ";
                // for (bool f : cc_flag) std::cout << f << " ";
                // std::cout << std::endl;
                long cc_to_count = 0;
                for (int j = 0; j < cc_flag.size(); ++j) {
                    if (cc_flag[j] == true) cc_to_count++;
                }
                vertex_cp[i] += cc_to_count;
            }
            ccav_vector_vertex[i] = ccav;
        }
        // else {
        //     std::cout << "vertex " << i << " is not on boundary. The adjacent faces are: ";
        //     std::vector<long> adj_tets = adj_tets_of_vertex(m, i);
        //     for (auto j : adj_tets) std::cout << j << " ";
        //     std::cout << std::endl;
        // }
    }
    // for (long j : vertex_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 3: assign new id to each vertex
    int counter = 0;
    std::vector<std::vector<long>> new_id_of_vertex(nb_vertex, std::vector<long>());
    for (int i = 0; i < nb_vertex; ++i) {
        if (vertex_cp[i] == 0) {
            new_id_of_vertex[i].push_back(counter);
            counter++;
            continue;
        }
        for (int j = 0; j < vertex_cp[i]; ++j) {
            new_id_of_vertex[i].push_back(counter);
            counter++;
        }
    }

    // Step 4: reconstruct the mesh
    wmtk::TetMesh mesh;
    wmtk::RowVectors4l tris;
    tris.resize(nb_tet, 4);
    for (long i = 0; i < nb_tet; ++i) {
        std::vector<wmtk::Tuple> list = wmtk::simplex::faces_single_dimension(
            m,
            wmtk::Simplex::tetrahedron(tets[i]),
            PrimitiveType::Vertex);
        std::vector<long> data(4, -1);
        for (int index = 0; index < 4; ++index) {
            long id_v = find_vertex_index(m, list[index]);
            if (vertex_cp[id_v] == 1)
                data[index] = new_id_of_vertex[id_v][0];
            else {
                for (long j = 0; j < ccav_vector_vertex[id_v].size(); ++j) {
                    if (std::find(
                            ccav_vector_vertex[id_v][j].begin(),
                            ccav_vector_vertex[id_v][j].end(),
                            i) != ccav_vector_vertex[id_v][j].end()) {
                        data[index] = new_id_of_vertex[id_v][j];
                        break;
                    }
                }
            }
        }
        // std::cout << "data = " << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3]
        //           << std::endl;
        tris.row(i) << data[0], data[1], data[2], data[3];
    }
    mesh.initialize(tris); // init the topology
    return mesh;
    // return m;
}
} // namespace wmtk::components::internal