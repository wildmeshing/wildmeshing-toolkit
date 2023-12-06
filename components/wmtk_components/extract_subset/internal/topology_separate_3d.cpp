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

    std::vector<std::vector<long>> adj_list_tets(nb_tet, std::vector<long>());
    for (long i = 0; i < nb_tet; ++i) {
        for (long j = i; j < nb_tet; ++j) {
            long face_con =
                edge_connected(m, wmtk::Simplex::tetrahedron(tets[i]), wmtk::Simplex::tetrahedron(tets[j]));
            if (face_con != -1) {
                adj_list_tets[i].push_back(j);
                adj_list_tets[j].push_back(i);
            }
        }
    }

    // Step 1: constuct a list of edge-connected components
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
    // print_vv(face_cc_list);
    std::vector<long> nb_cc_vec(nb_cc);
    for (long i = 0; i < nb_cc; ++i) nb_cc_vec[i] = face_cc_list[i].size();

    // Step 2: for each edge on the boundary, count number of group tets around it
    std::map<long, std::vector<std::vector<long>>> ccav_vector;
    for (long i = 0; i < nb_edge; ++i) {
        if (m.is_boundary(edges[i], PrimitiveType::Edge)) {
            // std::cout << "vertex " << i << " is on boundary. The adjacent faces are: ";
            
            // std::vector<long> adj_tets = adj_tets_of_edge(m, i);
            
            // for (auto j : adj_faces) std::cout << j << " ";
            // std::cout << std::endl;

            // std::vector<std::vector<long>> ccav = cc_around_vertex(m, adj_faces, adj_list_faces);
            // edge_cp[i] = ccav.size();
            // ccav_vector[i] = ccav;
        }
    }

    return m;
}
} // namespace wmtk::components::internal