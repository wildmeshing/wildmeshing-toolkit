#include "topology_separate_2d.hpp"
#include <iostream>
namespace wmtk::components::internal {

long edge_connected(const wmtk::TriMesh& m, Simplex i, Simplex j)
{
    auto edges = m.get_all(wmtk::PrimitiveType::Edge);
    auto i_tuple_list = wmtk::simplex::faces_single_dimension(m, i, PrimitiveType::Edge);
    auto j_tuple_list = wmtk::simplex::faces_single_dimension(m, j, PrimitiveType::Edge);
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            if (m.simplices_are_equal(
                    wmtk::Simplex::edge(i_tuple_list[a]),
                    wmtk::Simplex::edge(j_tuple_list[b])))
                return find_edge_index(m, i_tuple_list[a]);
        }
    }
    return -1;
}

long vertex_connected(const wmtk::TriMesh& m, Simplex i, Simplex j)
{
    if (edge_connected(m, i, j) == -1) return -1;
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    auto i_tuple_list = wmtk::simplex::faces_single_dimension(m, i, PrimitiveType::Vertex);
    auto j_tuple_list = wmtk::simplex::faces_single_dimension(m, j, PrimitiveType::Vertex);
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            if (m.simplices_are_equal(
                    wmtk::Simplex::vertex(i_tuple_list[a]),
                    wmtk::Simplex::vertex(j_tuple_list[b])))
                return find_vertex_index(m, i_tuple_list[a]);
        }
    }
    return -1;
}

long find_edge_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    auto edges = m.get_all(wmtk::PrimitiveType::Edge);
    for (int i = 0; i < edges.size(); ++i) {
        if (m.simplices_are_equal(wmtk::Simplex::edge(edges[i]), wmtk::Simplex::edge(t))) {
            return i;
        }
    }
    return -1;
}

long find_vertex_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    for (int i = 0; i < vertices.size(); ++i) {
        if (m.simplices_are_equal(wmtk::Simplex::vertex(vertices[i]), wmtk::Simplex::vertex(t))) {
            return i;
        }
    }
    return -1;
}

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m)
{
    // TODO: implement the algorithm proposed on Nov 2 meeting
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
            if nb_cp = 1
            then assign vertex with a singleton queue of current counter value, and increment the
    counter
            else assign vertex with a queue(FIFO) {ctr, ctr + 1, ..., ctr + nb_cp - 1}, increment
    the counter by nb_cp
    4. for each original triangle face:
            for each "real" vertex:
                if vertex has a singleton, use the id in there, then delete it
                else pop the queue out by 1, use this id in place of the original (assign to this
    vertex), delete it if empty
        CHECK: the map should be empty by now, no value queues left.
        NOTE: By saying "use", I mean to create an Eigen matrix and assign the id to the
    corresponding row
    5. return the new mesh
    */
    int nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    int nb_tri = m.capacity(wmtk::PrimitiveType::Face);
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    std::vector<std::vector<int>> face_adj_matrix(nb_tri, std::vector<int>(nb_tri, 0));
    std::vector<int> vertex_id(nb_vertex, -1); // reconstructed new id of a vertex
    std::vector<int> vertex_cp(nb_vertex, 1); // how many copies should we make on this vertex

    // Step 1: constuct a list of edge-connected components
    std::vector<std::vector<int>> face_cc_list;
    std::cout << "Hello 1 from topology_separate_2d!\n";
    for (int i = 0; i < nb_tri; ++i) {
        if (face_cc_list.size() == 0) {
            face_cc_list.push_back({i});
            continue;
        }
        bool connected_to_prev = false;
        for (auto j : face_cc_list) {
            // if (connected_to_prev) break;
            for (int k = 0; k < j.size(); ++k) {
                if (edge_connected(
                        m,
                        wmtk::Simplex::face(faces[i]),
                        wmtk::Simplex::face(faces[j[k]])) != -1) {
                    j.push_back(i);
                    break;
                }
            }
            if (*j.end() == i) {
                connected_to_prev = true;
                break;
            }
        }
        if (!connected_to_prev) face_cc_list.push_back({i});
    }

    std::cout << "Hello 2 from topology_separate_2d!\n";
    long nb_cc = face_cc_list.size();
    std::vector<int> nb_cc_vec(nb_cc);
    for (int i = 0; i < nb_cc; ++i) nb_cc_vec[i] = face_cc_list[i].size();

    // Step 2: for each component, count number of group tris around a singular vertex
    for (long i = 0; i < nb_cc; ++i) {
        std::set<long> connecting_vertices;
        std::map<long, std::vector<std::vector<long>>> connecting_tris;
        for (long j = 0; j < nb_cc_vec[i]; ++j) {
            for (long k = j + 1; k < nb_cc_vec[i]; ++k) {
                auto face1 = wmtk::Simplex::face(faces[face_cc_list[i][j]]);
                auto face2 = wmtk::Simplex::face(faces[face_cc_list[i][k]]);
                if (edge_connected(m, face1, face2) == -1) {
                    long id_v = vertex_connected(m, face1, face2);
                    if (id_v != -1) {
                        // if this vertex is not already in the set of candidates, add it
                        if (connecting_vertices.find(id_v) == connecting_vertices.end()) {
                            connecting_vertices.insert(id_v);
                            connecting_tris[id_v].push_back({{j}, {k}});
                        }
                        // if already in the set of candidates, check whether this pair of faces is
                        // in any existing group
                        else {
                            bool con_to_prev_j = false;
                            bool con_to_prev_k = false;
                            for (auto s : connecting_tris[id_v]) {
                                for (int t = 0; t < s.size(); ++t) {
                                    if (edge_connected(
                                            m,
                                            wmtk::Simplex::face(faces[face_cc_list[i][j]]),
                                            wmtk::Simplex::face(faces[s[t]])) != -1) {
                                        s.push_back(j);
                                        break;
                                    }
                                }
                                if (*s.end() == j) {
                                    con_to_prev_j = true;
                                    break;
                                }
                            }
                            if (!con_to_prev_j) connecting_tris[id_v].push_back({j});

                            for (auto s : connecting_tris[id_v]) {
                                for (int t = 0; t < s.size(); ++t) {
                                    if (edge_connected(
                                            m,
                                            wmtk::Simplex::face(faces[face_cc_list[i][k]]),
                                            wmtk::Simplex::face(faces[s[t]])) != -1) {
                                        s.push_back(k);
                                        break;
                                    }
                                }
                                if (*s.end() == k) {
                                    con_to_prev_k = true;
                                    break;
                                }
                            }
                            if (!con_to_prev_k) connecting_tris[id_v].push_back({k});
                        }
                    }
                }
            }
        }
        // incremant nb_cp of each connecting vertex
        for (auto id_v : connecting_vertices) {
            vertex_cp[id_v] += connecting_tris[id_v].size();
        }
    }

    // Step 3: assign queues to each vertex
    std::cout << "Hello 3 from topology_separate_2d!\n";
    int counter = 0;
    std::vector<std::queue<long>> queues_of_vertex(nb_vertex, std::queue<long>());
    for (int i = 0; i < nb_vertex; ++i) {
        if (vertex_cp[i] == 1) {
            queues_of_vertex[i].push(counter);
            counter++;
        } else {
            for (int j = 0; j < vertex_cp[i]; ++j) {
                queues_of_vertex[i].push(counter);
                counter++;
            }
        }
    }

    // Step 4: reconstruct the mesh
    wmtk::TriMesh mesh;
    wmtk::RowVectors3l tris;
    tris.resize(nb_tri, 3);
    for (long i = 0; i < nb_tri; ++i) {
        auto list = wmtk::simplex::faces_single_dimension(
            m,
            wmtk::Simplex::face(faces[i]),
            PrimitiveType::Vertex);
        std::vector<long> data(3, -1);
        for (int index = 0; index < 3; ++index) {
            long id_v = find_vertex_index(m, list[index]);
            data[index] = queues_of_vertex[id_v].front();
            queues_of_vertex[id_v].pop();
        }
        tris.row(i) << data[0], data[1], data[2];
    }
    std::cout << "Hello 4 from topology_separate_2d!\n";
    mesh.initialize(tris); // init the topology
    std::cout << "Hello 5 from topology_separate_2d!\n";

    // CHECK: the map should be empty by now, no value queues left.
    for (long i = 0; i < nb_vertex; ++i) {
        if (queues_of_vertex[i].size() != 0) {
            std::runtime_error("ERROR: queue not empty!");
        }
    }
    // return mesh;
    return m;
}
} // namespace wmtk::components::internal
