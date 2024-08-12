#include "is_free.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>
namespace wmtk::tests {
namespace {}
bool is_free(const Mesh& m)
{
    switch (m.top_simplex_type()) {
    case PrimitiveType::Tetrahedron: return is_free(dynamic_cast<const TetMesh&>(m));
    case PrimitiveType::Triangle: return is_free(dynamic_cast<const TriMesh&>(m));
    case PrimitiveType::Edge: return is_free(dynamic_cast<const EdgeMesh&>(m));
    case PrimitiveType::Vertex: // return is_free(dynamic_cast<const PointMesh&>(m));
    default: throw std::runtime_error("Not implemented");
    }
    //
    return false;
}
bool is_free(const EdgeMesh& m)
{
    assert(m.is_connectivity_valid());
    size_t edge_count = m.get_all(PrimitiveType::Edge).size();
    size_t vertex_count = m.get_all(PrimitiveType::Vertex).size();
    //
    if (2 * edge_count != vertex_count) {
        logger().warn(
            "Free edge mesh with {} edges should have {} vertices, got {}",
            edge_count,
            2 * edge_count,
            vertex_count);
        return false;
    }
    return true;
}
bool is_free(const TriMesh& m)
{
    assert(m.is_connectivity_valid());
    size_t triangle_count = m.get_all(PrimitiveType::Triangle).size();
    size_t edge_count = m.get_all(PrimitiveType::Edge).size();
    size_t vertex_count = m.get_all(PrimitiveType::Vertex).size();
    //
    if (3 * triangle_count != vertex_count) {
        logger().warn(
            "Free triangle mesh with {} triangles should have {} vertices, got {}",
            triangle_count,
            3 * triangle_count,
            vertex_count);
        return false;
    }
    if (3 * triangle_count != edge_count) {
        logger().warn(
            "Free triangle mesh with {} triangles should have {} vertices, got {}",
            triangle_count,
            3 * triangle_count,
            edge_count);
        return false;
    }
    return true;
    //
}
bool is_free(const TetMesh& m)
{
    //
    assert(m.is_connectivity_valid());
    size_t tetrahedron_count = m.get_all(PrimitiveType::Tetrahedron).size();
    size_t triangle_count = m.get_all(PrimitiveType::Triangle).size();
    size_t edge_count = m.get_all(PrimitiveType::Edge).size();
    size_t vertex_count = m.get_all(PrimitiveType::Vertex).size();
    //
    if (4 * tetrahedron_count != vertex_count) {
        logger().warn(
            "Free tetrahedron mesh with {} tetrahedrons should have {} vertices, got {}",
            tetrahedron_count,
            4 * tetrahedron_count,
            vertex_count);
        return false;
    }
    if (6 * tetrahedron_count != edge_count) {
        logger().warn(
            "Free tetrahedron mesh with {} tetrahedrons should have {} vertices, got {}",
            tetrahedron_count,
            6 * tetrahedron_count,
            edge_count);
        return false;
    }
    if (4 * tetrahedron_count != triangle_count) {
        logger().warn(
            "Free tetrahedron mesh with {} tetrahedrons should have {} vertices, got {}",
            tetrahedron_count,
            4 * tetrahedron_count,
            triangle_count);
        return false;
    }
    return true;
}
} // namespace wmtk::tests
