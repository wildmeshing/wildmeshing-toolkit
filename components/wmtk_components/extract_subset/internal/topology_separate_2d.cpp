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

long find_face_index(const wmtk::TriMesh& m, wmtk::Tuple t)
{
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    for (int i = 0; i < faces.size(); ++i) {
        if (m.simplices_are_equal(wmtk::Simplex::face(faces[i]), wmtk::Simplex::face(t))) {
            return i;
        }
    }
    return -1;
}

std::vector<long> adj_faces_of_vertex(const wmtk::TriMesh& m, long i)
{
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    std::vector<long> adj_faces;
    for (auto face : faces) {
        auto face_vertices = wmtk::simplex::faces_single_dimension(
            m,
            wmtk::Simplex::face(face),
            PrimitiveType::Vertex);
        for (auto vertex : face_vertices) {
            if (m.simplices_are_equal(
                    wmtk::Simplex::vertex(vertex),
                    wmtk::Simplex::vertex(vertices[i]))) {
                adj_faces.push_back(find_face_index(m, face));
                break;
            }
        }
    }
    return adj_faces;
}

void get_edge_count(const wmtk::TriMesh& m, std::vector<int>& edge_count)
{
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    for (auto tri : faces) {
        auto edges =
            wmtk::simplex::faces_single_dimension(m, wmtk::Simplex::face(tri), PrimitiveType::Edge);
        for (auto edge : edges) {
            edge_count[find_edge_index(m, edge)]++;
        }
    }
}

bool on_boundary(const wmtk::TriMesh& m, long i)
{
    // TODO: implement this, check if a vertex is on the boundary
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto nb_edges = m.capacity(wmtk::PrimitiveType::Edge);
    auto s = wmtk::Simplex::vertex(m.get_all(wmtk::PrimitiveType::Vertex)[i]);
    std::vector<int> edge_count(nb_edges, 0);
    get_edge_count(m, edge_count);

    // std::cout << "Edge Count: ";
    // for (int count : edge_count) std::cout << count << " ";
    // std::cout << std::endl;

    auto adj_faces = adj_faces_of_vertex(m, i);
    // for (auto index : adj_faces) std::cout << "face " << index << " contains vertex " << i <<
    // "\n";

    for (auto index : adj_faces) {
        auto face = wmtk::Simplex::face(faces[index]);
        auto edges = wmtk::simplex::faces_single_dimension(m, face, PrimitiveType::Edge);
        for (auto edge : edges) {
            // std::cout << "edge # " << find_edge_index(m, edge);
            auto edge_vertices = wmtk::simplex::faces_single_dimension(
                m,
                wmtk::Simplex::edge(edge),
                PrimitiveType::Vertex);

            for (auto edge_vertex : edge_vertices) {
                // std::cout << ", vertex " << find_vertex_index(m, edge_vertex.tuple()) << " in ";
                if (m.simplices_are_equal(wmtk::Simplex::vertex(edge_vertex), s)) {
                    // std::cout << "edge " << find_edge_index(m, edge) << " in face " << index
                    //   << " contains vertex " << i << "\n";
                    if (edge_count[find_edge_index(m, edge)] % 2 == 1) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


void print_vv(std::vector<std::vector<long>>& vv)
{
    for (auto i : vv) {
        std::cout << i.size() << ": ";
        for (auto j : i) {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }
}

void dfs(
    long start,
    std::vector<bool>& visited,
    std::vector<long>& cc,
    const std::vector<std::vector<long>>& adj,
    const std::function<bool(long, std::vector<long>&)>& condition,
    std::vector<long>& candidates)
{
    visited[start] = true;
    cc.push_back(start);
    for (auto j : adj[start]) {
        if (!visited[j] && condition(j, candidates)) {
            dfs(j, visited, cc, adj, condition, candidates);
        }
    }
}

long calc_nb_groups(
    const wmtk::TriMesh& m,
    std::vector<long>& adj_faces,
    std::vector<std::vector<long>>& adj_list_faces)
{
    // use exactly the same also as finding connected components in the whole mesh to find cc here

    std::vector<std::vector<long>> face_cc_list;
    std::vector<bool> visited_faces(m.capacity(wmtk::PrimitiveType::Face), false);
    auto condition = [](long face, std::vector<long>& candidates) {
        return std::find(candidates.begin(), candidates.end(), face) != candidates.end();
    };
    for (long i = 0; i < adj_faces.size(); ++i) {
        // std::cout << "adj_faces[i] = " << adj_faces[i] << std::endl;
        if (visited_faces[adj_faces[i]] == false) {
            std::vector<long> cc;
            dfs(adj_faces[i], visited_faces, cc, adj_list_faces, condition, adj_faces);
            face_cc_list.push_back(cc);
        }
    }
    return face_cc_list.size();
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
    long nb_vertex = m.capacity(wmtk::PrimitiveType::Vertex);
    long nb_tri = m.capacity(wmtk::PrimitiveType::Face);
    auto faces = m.get_all(wmtk::PrimitiveType::Face);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    std::vector<long> vertex_id(nb_vertex, -1); // reconstructed new id of a vertex
    std::vector<long> vertex_cp(nb_vertex, 1); // how many copies should we make on this vertex

    std::cout << "# of tris = " << nb_tri << std::endl;
    std::cout << "# of vertices = " << nb_vertex << std::endl;

    // Prior work: build an face-adjacency list of triangle faces
    std::vector<std::vector<long>> adj_list_faces(nb_tri, std::vector<long>());
    std::vector<std::vector<long>> adj_matrix_faces(nb_tri, std::vector<long>(nb_tri, -1));
    for (long i = 0; i < nb_tri; ++i) {
        for (long j = i; j < nb_tri; ++j) {
            long edge_con =
                edge_connected(m, wmtk::Simplex::face(faces[i]), wmtk::Simplex::face(faces[j]));
            if (edge_con != -1) {
                adj_list_faces[i].push_back(j);
                adj_list_faces[j].push_back(i);
                adj_matrix_faces[i][j] = edge_con;
                adj_matrix_faces[j][i] = edge_con;
            }
        }
    }

    // Step 1: constuct a list of edge-connected components
    std::vector<std::vector<long>> face_cc_list;
    std::vector<bool> visited_faces(nb_tri, false);
    auto condition = [](long face, std::vector<long>&) { return true; };
    auto nullvector = std::vector<long>(nb_tri);
    for (long i = 0; i < nb_tri; ++i) nullvector[i] = i;
    for (long i = 0; i < nb_tri; ++i) {
        if (visited_faces[i] == false) {
            std::vector<long> cc;
            dfs(i, visited_faces, cc, adj_list_faces, condition, nullvector);
            face_cc_list.push_back(cc);
        }
    }
    long nb_cc = face_cc_list.size();
    std::cout << "# of cc = " << nb_cc << std::endl;
    print_vv(face_cc_list);
    std::vector<long> nb_cc_vec(nb_cc);
    for (long i = 0; i < nb_cc; ++i) nb_cc_vec[i] = face_cc_list[i].size();

    // Step 2: for each vertex on the boundary, count number of group tris around it

    // start a version of the algo where we loop over vertices instead of triangles
    for (long i = 0; i < nb_vertex; ++i) {
        if (on_boundary(m, i)) {
            // std::cout << "vertex " << i << " is on boundary. The adjacent faces are: ";
            std::vector<long> adj_faces = adj_faces_of_vertex(m, i);
            // for (auto j : adj_faces) std::cout << j << " ";
            // std::cout << std::endl;
            vertex_cp[i] = calc_nb_groups(m, adj_faces, adj_list_faces);
        }
    }
    // for (auto j : vertex_cp) std::cout << j << " ";
    // std::cout << std::endl;

    // Step 3: assign queues to each vertex
    std::cout << "Hello 1 from topology_separate_2d!\n";
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

    // // Step 4: reconstruct the mesh
    // wmtk::TriMesh mesh;
    // wmtk::RowVectors3l tris;
    // tris.resize(nb_tri, 3);
    // for (long i = 0; i < nb_tri; ++i) {
    //     auto list = wmtk::simplex::faces_single_dimension(
    //         m,
    //         wmtk::Simplex::face(faces[i]),
    //         PrimitiveType::Vertex);
    //     std::vector<long> data(3, -1);
    //     for (int index = 0; index < 3; ++index) {
    //         long id_v = find_vertex_index(m, list[index]);
    //         data[index] = queues_of_vertex[id_v].front();
    //         queues_of_vertex[id_v].pop();
    //     }
    //     std::cout << "data = " << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
    //     tris.row(i) << data[0], data[1], data[2];
    // }
    // std::cout << "Hello 3 from topology_separate_2d!\n";
    // mesh.initialize(tris); // init the topology
    // std::cout << "Hello 4 from topology_separate_2d!\n";

    // // CHECK: the map should be empty by now, no value queues left.
    // for (long i = 0; i < nb_vertex; ++i) {
    //     if (queues_of_vertex[i].size() != 0) {
    //         std::runtime_error("ERROR: queue not empty!");
    //     }
    // }
    // return mesh;
    return m;
}
} // namespace wmtk::components::internal
