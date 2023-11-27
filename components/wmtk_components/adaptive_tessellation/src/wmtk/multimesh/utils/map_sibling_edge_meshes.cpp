#include "map_sibling_edge_meshes.hpp"

namespace wmtk::multimesh::utils {
std::map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> map_sibling_edge_meshes(
    const Mesh& position_mesh)
{
    std::map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> sibling_edge_meshes;
    std::vector<Tuple> edge_tuples = position_mesh.get_all(PrimitiveType::Edge);
    std::vector<std::shared_ptr<Mesh>> uv_meshes = position_mesh.get_child_meshes();
    for (const Tuple& e : edge_tuples) {
        if (position_mesh.is_boundary(e, PrimitiveType::Edge)) {
            std::vector<std::shared_ptr<Mesh>> seam_edge_meshes;
            for (const auto& uv_mesh : uv_meshes) {
                // the edge is a seam edge in uv mesh
                auto uv_mesh_edges = position_mesh.map(*uv_mesh, wmtk::Simplex::edge(e));
                if (uv_mesh_edges.size() > 0) {
                    std::vector<std::shared_ptr<Mesh>> edge_meshes = uv_mesh->get_child_meshes();
                    wmtk::Simplex uv_mesh_e = uv_mesh_edges[0];
                    for (long i = 0; (*uv_mesh).map(*edge_meshes[i], uv_mesh_e).size() > 0; ++i) {
                        seam_edge_meshes.push_back(edge_meshes[i]);
                    }
                }
            }
            if (seam_edge_meshes.size() > 0) {
                assert(seam_edge_meshes.size() == 2);
                sibling_edge_meshes[seam_edge_meshes[0]] = seam_edge_meshes[1];
                sibling_edge_meshes[seam_edge_meshes[1]] = seam_edge_meshes[0];
            }
        }
    }
    return sibling_edge_meshes;
}
} // namespace wmtk::multimesh::utils