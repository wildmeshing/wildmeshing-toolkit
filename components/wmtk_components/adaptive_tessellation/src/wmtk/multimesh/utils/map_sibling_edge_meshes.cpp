#include "map_sibling_edge_meshes.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::adaptive_tessellation::multimesh::utils {
std::map<EdgeMesh*, EdgeMesh*> map_sibling_edge_meshes(const EdgeMesh& position_mesh)
{
    std::map<EdgeMesh*, EdgeMesh*> sibling_edge_meshes;
    std::vector<Tuple> edge_tuples = position_mesh.get_all(PrimitiveType::Edge);
    std::vector<std::shared_ptr<Mesh>> uv_meshes = position_mesh.get_child_meshes();
    assert(uv_meshes.size() == 1);
    Mesh& uv_mesh_m = *uv_meshes[0];
    TriMesh& uv_mesh = static_cast<TriMesh&>(uv_mesh_m);

    for (const Tuple& e : edge_tuples) {
        std::vector<std::shared_ptr<Mesh>> seam_edge_meshes;
        auto uv_mesh_edges = position_mesh.map(uv_mesh, wmtk::Simplex::edge(e));
        if (uv_mesh_edges.size() > 1) { // the edge is a seam edge in uv mesh
            assert(uv_mesh_edges.size() == 2);
            // assuming a seam segment consists of only two edges
            std::vector<std::shared_ptr<Mesh>> edge_meshes = uv_mesh.get_child_meshes();
            wmtk::Simplex seam_edge0 = uv_mesh_edges[0];
            wmtk::Simplex seam_edge1 = uv_mesh_edges[1];
            for (long i = 0; i < edge_meshes.size(); ++i) {
                auto edge_mesh_seam0 = uv_mesh.map(*edge_meshes[i], seam_edge0);
                auto edge_mesh_seam1 = uv_mesh.map(*edge_meshes[i], seam_edge1);
                if (edge_mesh_seam0.size() > 0 || edge_mesh_seam1.size() > 0) {
                    seam_edge_meshes.emplace_back(edge_meshes[i]);
                }
            }
        }
        if (seam_edge_meshes.size() > 0) {
            assert(seam_edge_meshes.size() == 2);
            // assuming a seam segment consists of only two edges
            Mesh* sibling0_m = seam_edge_meshes[0].get();
            Mesh* sibling1_m = seam_edge_meshes[1].get();
            EdgeMesh* sibling0 = static_cast<EdgeMesh*>(sibling0_m);
            EdgeMesh* sibling1 = static_cast<EdgeMesh*>(sibling1_m);
            sibling_edge_meshes[sibling1] = sibling0;
            sibling_edge_meshes[sibling0] = sibling1;
        }
    }
    return sibling_edge_meshes;
}

std::map<EdgeMesh*, EdgeMesh*> map_sibling_edge_meshes(
    const std::vector<std::shared_ptr<EdgeMesh>> edge_meshes)
{
    std::map<EdgeMesh*, EdgeMesh*> sibling_edge_meshes;
    for (long i = 0; i < edge_meshes.size(); ++i) {
        auto edge_meshi = edge_meshes[i];
        Tuple edge = edge_meshi->get_all(PrimitiveType::Edge)[0];
        // construct a simplex of an arbitray edge from the edge mesh
        for (long j = i + 1; j < edge_meshes.size(); ++j) {
            auto edge_meshj = edge_meshes[j];
            if (edge_meshi->map(*edge_meshj, Simplex::edge(edge)).size() > 0) {
                EdgeMesh* ei = static_cast<EdgeMesh*>(edge_meshi.get());
                EdgeMesh* ej = static_cast<EdgeMesh*>(edge_meshj.get());
                sibling_edge_meshes[ei] = ej;
                sibling_edge_meshes[ej] = ei;
            }
        }
    }
    return sibling_edge_meshes;
}
} // namespace wmtk::components::adaptive_tessellation::multimesh::utils
