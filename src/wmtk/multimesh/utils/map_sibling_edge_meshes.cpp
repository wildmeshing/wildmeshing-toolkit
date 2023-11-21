#include "map_sibling_edge_meshes.hpp"

namespace wmtk::multimesh::utils {
std::map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> map_sibling_edge_meshes(
    const Mesh& position_mesh)
{
    std::map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> sibling_edge_meshes;
    std::vector<Tuple> edge_tuples = position_mesh.get_all(PrimitiveType::Edge);
    auto uv_meshes = position_mesh.get_child_meshes();
    for (const Tuple& e : edge_tuples) {
        if (position_mesh.is_boundary(e, PrimitiveType::Edge)) {
            for (const auto& uv_mesh : uv_meshes) {
                // the edge is a seam edge in uv mesh
                if (position_mesh.map(*uv_mesh, wmtk::Simplex::edge(e)).size() > 0) {
                    auto edge_meshes = uv_mesh->get_child_meshes();
                    assert(edge_meshes.size() == 2);
                    sibling_edge_meshes.insert(std::make_pair(edge_meshes[0], edge_meshes[1]));
                    sibling_edge_meshes.insert(std::make_pair(edge_meshes[1], edge_meshes[0]));
                }
            }
        }
    }

    return sibling_edge_meshes;
}
} // namespace wmtk::multimesh::utils