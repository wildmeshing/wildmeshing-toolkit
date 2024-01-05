#include "map_sibling_edge_meshes.hpp"

namespace wmtk::components::adaptive_tessellation::multimesh::utils {
std::map<Mesh*, Mesh*> map_sibling_edge_meshes(const Mesh& position_mesh)
{
    std::map<Mesh*, Mesh*> sibling_edge_meshes;
    std::vector<Tuple> edge_tuples = position_mesh.get_all(PrimitiveType::Edge);
    std::vector<std::shared_ptr<Mesh>> uv_meshes = position_mesh.get_child_meshes();
    assert(uv_meshes.size() == 1);
    Mesh* uv_mesh = uv_meshes[0].get();
    for (const Tuple& e : edge_tuples) {
        std::vector<std::shared_ptr<Mesh>> seam_edge_meshes;
        auto uv_mesh_edges = position_mesh.map(*uv_mesh, wmtk::simplex::Simplex::edge(e));
        if (uv_mesh_edges.size() > 1) { // the edge is a seam edge in uv mesh
            assert(uv_mesh_edges.size() == 2);
            // assuming a seam segment consists of only two edges
            std::vector<std::shared_ptr<Mesh>> edge_meshes = (*uv_mesh).get_child_meshes();
            wmtk::simplex::Simplex seam_edge0 = uv_mesh_edges[0];
            wmtk::simplex::Simplex seam_edge1 = uv_mesh_edges[1];
            for (int64_t i = 0; i < edge_meshes.size(); ++i) {
                auto edge_mesh_seam0 = (*uv_mesh).map(*edge_meshes[i], seam_edge0);
                auto edge_mesh_seam1 = (*uv_mesh).map(*edge_meshes[i], seam_edge1);
                if (edge_mesh_seam0.size() > 0 || edge_mesh_seam1.size() > 0) {
                    seam_edge_meshes.emplace_back(edge_meshes[i]);
                }
            }
        }
        if (seam_edge_meshes.size() > 0) {
            assert(seam_edge_meshes.size() == 2);
            // assuming a seam segment consists of only two edges
            Mesh* sibling0 = seam_edge_meshes[0].get();
            Mesh* sibling1 = seam_edge_meshes[1].get();
            sibling_edge_meshes[sibling1] = sibling0;
            sibling_edge_meshes[sibling0] = sibling1;
        }
    }
    return sibling_edge_meshes;
}

std::map<Mesh*, Mesh*> map_sibling_edge_meshes(const std::vector<std::shared_ptr<Mesh>> edge_meshes)
{
    std::map<Mesh*, Mesh*> sibling_edge_meshes;
    for (int64_t i = 0; i < edge_meshes.size(); ++i) {
        Mesh* edge_meshi = edge_meshes[i].get();
        Tuple edge = edge_meshi->get_all(PrimitiveType::Edge)[0];
        // construct a simplex of an arbitray edge from the edge mesh
        for (int64_t j = i + 1; j < edge_meshes.size(); ++j) {
            Mesh* edge_meshj = edge_meshes[j].get();
            if (edge_meshi->map(*edge_meshj, simplex::Simplex::edge(edge)).size() > 0) {
                sibling_edge_meshes[edge_meshi] = edge_meshj;
                sibling_edge_meshes[edge_meshj] = edge_meshi;
            }
        }
    }
    return sibling_edge_meshes;
}
} // namespace wmtk::components::adaptive_tessellation::multimesh::utils