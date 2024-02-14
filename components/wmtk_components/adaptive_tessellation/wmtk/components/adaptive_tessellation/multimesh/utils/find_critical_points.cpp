#include "find_critical_points.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>

using namespace wmtk;
using namespace wmtk::simplex;
namespace wmtk::components::multimesh::utils {
std::set<Tuple> find_critical_points(const Mesh& uv_mesh, const Mesh& position_mesh)
{
    std::set<Tuple> critical_vids;
    // iterate over all vertices in the boudnary of uv mesh
    std::vector<Tuple> vertex_tuples = uv_mesh.get_all(PrimitiveType::Vertex);
    for (const Tuple& v : vertex_tuples) {
        if (!uv_mesh.is_boundary(v, PrimitiveType::Vertex)) {
            continue;
        }
        // get the corresponding vertex in position mesh
        std::vector<Simplex> position_vertices = uv_mesh.map(position_mesh, Simplex::vertex(v));
        assert(position_vertices.size() == 1);
        Tuple p = position_vertices[0].tuple();
        // get the corresponding vertices in uv mesh
        std::vector<Simplex> uv_vertices = position_mesh.map(uv_mesh, Simplex::vertex(p));
        assert(uv_vertices.size() > 0);
        if (position_mesh.is_boundary(p, PrimitiveType::Vertex)) {
            if (uv_vertices.size() != 1) {
                critical_vids.insert(v);
            }
        } else {
            if (uv_vertices.size() != 2) {
                critical_vids.insert(v);
            }
        }
    }

    return critical_vids;
}
} // namespace wmtk::components::multimesh::utils