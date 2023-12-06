#include "topology_separate_3d.hpp"

namespace wmtk::components::internal {

wmtk::TetMesh topology_separate_3d(wmtk::TetMesh m)
{
    long nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    long nb_tet = m.capacity(wmtk::PrimitiveType::Tetrahedron);
    long nb_edge = m.capacity(wmtk::PrimitiveType::Edge);
    std::vector<wmtk::Tuple> edges = m.get_all(wmtk::PrimitiveType::Edge);
    std::vector<wmtk::Tuple> tets = m.get_all(wmtk::PrimitiveType::Tetrahedron);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    std::vector<long> edge_cp(nb_edge, 1); // how many copies should we make on this edge
    std::vector<long> vertex_cp(nb_vertex, 1); // how many copies should we make on this vertex

    // std::cout << "# of tets = " << nb_tet << std::endl;
    // std::cout << "# of edges = " << nb_edge << std::endl;
    // std::cout << "# of vertices = " << nb_vertex << std::endl;

    // Prior work: build an face-adjacency list of tets
    std::vector<std::vector<long>> adj_list_tets(nb_tet, std::vector<long>());
    for (long i = 0; i < nb_tet; ++i) {
        for (long j = i; j < nb_tet; ++j) {
            long face_con = face_connected(
                m,
                wmtk::Simplex::tetrahedron(tets[i]),
                wmtk::Simplex::tetrahedron(tets[j]));
            if (face_con != -1) {
                adj_list_tets[i].push_back(j);
                adj_list_tets[j].push_back(i);
            }
        }
    }

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

    // Step 2: for each edge on the boundary, count number of group tets around it
    std::map<long, std::vector<std::vector<long>>> ccav_vector;
    for (long i = 0; i < nb_edge; ++i) {
        if (m.is_boundary(edges[i], PrimitiveType::Edge)) {
            // std::cout << "vertex " << i << " is on boundary. The adjacent faces are: ";
            std::vector<long> adj_tets = adj_tets_of_edge(m, i);
            // for (auto j : adj_faces) std::cout << j << " ";
            // std::cout << std::endl;
            std::vector<std::vector<long>> ccav = cc_around_edge(m, adj_tets, adj_list_tets);
            edge_cp[i] = ccav.size();
            ccav_vector[i] = ccav;
        }
    }
    // for (auto j : vertex_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 3: assign queues to each edge
    int counter = 0;
    std::vector<std::vector<long>> new_id_of_edge(nb_edge, std::vector<long>());
    for (int i = 0; i < nb_edge; ++i) {
        for (int j = 0; j < edge_cp[i]; ++j) {
            new_id_of_edge[i].push_back(counter);
            counter++;
        }
    }

    return m;
}
} // namespace wmtk::components::internal