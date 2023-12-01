#include "utils.hpp"

namespace wmtk::components::internal {

template <typename M>
long find_index(
    const M& m,
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

template <typename M>
long find_edge_index(const M& m, wmtk::Tuple t)
{
    // std::function<wmtk::Simplex(const wmtk::Tuple&)>
    auto extractFunction = [](const wmtk::Tuple& tuple) { return wmtk::Simplex::edge(tuple); };
    return find_index<M>(m, t, extractFunction, wmtk::PrimitiveType::Edge);
}

template <typename M>
long find_vertex_index(const M& m, wmtk::Tuple t)
{
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractFunction =
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::vertex(tuple); };
    return find_index<M>(m, t, extractFunction, wmtk::PrimitiveType::Vertex);
}

template <typename M>
long find_face_index(const M& m, wmtk::Tuple t)
{
    return find_index<M>(
        m,
        t,
        [](const wmtk::Tuple& tuple) { return wmtk::Simplex::face(tuple); },
        wmtk::PrimitiveType::Face);
}

void get_edge_count(const wmtk::TriMesh& m, std::vector<bool>& edge_count)
{
    std::vector<wmtk::Tuple> faces = m.get_all(wmtk::PrimitiveType::Face);
    for (wmtk::Tuple tri : faces) {
        std::vector<wmtk::Tuple> edges =
            wmtk::simplex::faces_single_dimension(m, wmtk::Simplex::face(tri), PrimitiveType::Edge);
        for (wmtk::Tuple edge : edges) {
            long l = find_edge_index<wmtk::TriMesh>(m, edge);
            edge_count[l] = !edge_count[l];
        }
    }
}

bool vertex_on_boundary(const wmtk::TriMesh& m, std::vector<bool>& edge_count, long i)
{
    // Algo to determine whether a vertex is on the boundary:
    // 1. given a vertex, find all the faces adjacent to this vertex
    // 2. for each adj face, find all the 3 edges
    // 3. for each edge, find all the 2 vertices
    // 4. for each vertex, check if it is the same as the vertex we are checking
    // 5. if yes, then check if the current edge is on the boundary
    // i.e. edge appeared in the mesh for an odd number of times
    // 6. if edge on boundary, then return true
    // 7. in the end if if all edges in all adj faces are not on the boundary, then return false
    std::vector<wmtk::Tuple> faces = m.get_all(wmtk::PrimitiveType::Face);
    wmtk::simplex::Simplex s = wmtk::Simplex::vertex(m.get_all(wmtk::PrimitiveType::Vertex)[i]);
    std::vector<long> adj_faces = adj_faces_of_vertex(m, i);
    // for (auto index : adj_faces) std::cout << "face " << index << " contains vertex " << i <<
    // "\n";

    for (long index : adj_faces) {
        wmtk::simplex::Simplex face = wmtk::Simplex::face(faces[index]);
        std::vector<wmtk::Tuple> edges =
            wmtk::simplex::faces_single_dimension(m, face, PrimitiveType::Edge);
        for (wmtk::Tuple edge : edges) {
            // std::cout << "edge # " << find_edge_index<wmtk::TriMesh>(m, edge);
            std::vector<wmtk::Tuple> edge_vertices = wmtk::simplex::faces_single_dimension(
                m,
                wmtk::Simplex::edge(edge),
                PrimitiveType::Vertex);
            for (wmtk::Tuple edge_vertex : edge_vertices) {
                // std::cout << ", vertex " << find_vertex_index<wmtk::TriMesh>(m, edge_vertex) << "
                // in ";
                if (m.simplices_are_equal(wmtk::Simplex::vertex(edge_vertex), s)) {
                    // std::cout << "edge " << find_edge_index<wmtk::TriMesh>(m, edge) << " in face
                    // " << index
                    //           << " contains vertex " << i << "\n";
                    if (edge_count[find_edge_index<wmtk::TriMesh>(m, edge)]) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
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
                adj_faces.push_back(find_face_index<wmtk::TriMesh>(m, face));
                break;
            }
        }
    }
    return adj_faces;
}

} // namespace wmtk::components::internal