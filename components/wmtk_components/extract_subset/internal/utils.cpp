#include "utils.hpp"

namespace wmtk::components::internal {
template <typename Extractor>
long connected(
    const wmtk::TriMesh& m,
    wmtk::Simplex i,
    wmtk::Simplex j,
    Extractor extractor,
    wmtk::PrimitiveType type)
{
    std::vector<wmtk::Tuple> primitives = m.get_all(type);
    std::vector<wmtk::Tuple> i_tuple_list = wmtk::simplex::faces_single_dimension(m, i, type);
    std::vector<wmtk::Tuple> j_tuple_list = wmtk::simplex::faces_single_dimension(m, j, type);
    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            if (m.simplices_are_equal(extractor(i_tuple_list[a]), extractor(j_tuple_list[b]))) {
                return find_index(m, i_tuple_list[a], extractor, type);
            }
        }
    }
    return -1;
}

long edge_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j)
{
    return connected(
        m,
        i,
        j,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::edge(tuple); },
        wmtk::PrimitiveType::Edge);
}

long vertex_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j)
{
    return connected(
        m,
        i,
        j,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::vertex(tuple); },
        wmtk::PrimitiveType::Vertex);
}

long find_index(
    const wmtk::Mesh& m,
    wmtk::Tuple t,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractFunction,
    wmtk::PrimitiveType type)
{
    std::vector<wmtk::Tuple> primitives = m.get_all(type);
    for (int i = 0; i < primitives.size(); ++i) {
        if (m.simplices_are_equal(extractFunction(primitives[i]), extractFunction(t))) {
            return i;
        }
    }
    return -1;
}

long find_edge_index(const wmtk::Mesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::edge(tuple); },
        wmtk::PrimitiveType::Edge);
}

long find_vertex_index(const wmtk::Mesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::vertex(tuple); },
        wmtk::PrimitiveType::Vertex);
}

long find_face_index(const wmtk::Mesh& m, wmtk::Tuple t)
{
    return find_index(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::face(tuple); },
        wmtk::PrimitiveType::Face);
}

std::vector<long> adj_faces_of_vertex(const wmtk::TriMesh& m, long i)
{
    // Algo: given a vertex, traverse all faces in the mesh, for each face, find all the vertices
    // if the vertex we are checking is in the list, then add the face index to a list to return
    std::vector<wmtk::Tuple> faces = m.get_all(wmtk::PrimitiveType::Face);
    std::vector<wmtk::Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    std::vector<long> adj_faces;
    for (wmtk::Tuple face : faces) {
        std::vector<wmtk::Tuple> face_vertices = wmtk::simplex::faces_single_dimension(
            m,
            wmtk::Simplex::face(face),
            PrimitiveType::Vertex);
        for (wmtk::Tuple vertex : face_vertices) {
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
    for (long j : adj[start]) {
        if (!visited[j] && condition(j, candidates)) {
            dfs(j, visited, cc, adj, condition, candidates);
        }
    }
}

std::vector<std::vector<long>> cc_around_vertex(
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
    return face_cc_list;
}
} // namespace wmtk::components::internal