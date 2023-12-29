#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/AttributeTransferStrategy.hpp>

namespace wmtk::components::adaptive_tessellation::operations::internal {

class ATData
{
    std::shared_ptr<TriMesh> m_uv_mesh_ptr;
    std::shared_ptr<TriMesh> m_position_mesh_ptr;
    std::vector<std::shared_ptr<Mesh>> m_edge_mesh_ptrs;
    std::map<Mesh*, Mesh*> m_sibling_meshes_map;
    std::array<image::Image, 3>& m_images;

public:
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior
    // edge op input_m = uv_mesh, boundary edge op input_m = position_mesh) The invariant that
    // is shared among the operations besides the base invariants is the no-triangle-inversion
    // of the uv_mesh

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    MeshAttributeHandle<double> m_uv_handle;
    MeshAttributeHandle<double> m_uv_edge_length_handle;
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        m_edge_length_update;
    // Scheduler m_scheduler;

    ATData(
        std::shared_ptr<TriMesh> uv_mesh,
        std::shared_ptr<TriMesh> position_mesh,
        std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
        std::map<Mesh*, Mesh*> sibling_meshes_map,
        std::array<image::Image, 3>& images);
    ATData(
        std::shared_ptr<TriMesh> uv_mesh,
        std::shared_ptr<TriMesh> position_mesh,
        std::array<image::Image, 3>& images);


    MeshAttributeHandle<double>& uv_handle();
    TriMesh& uv_mesh() const;
    TriMesh& position_mesh() const;
    const Mesh& edge_mesh(long i) const;
    Mesh* sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr);
    Simplex sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s);
    const std::array<image::Image, 3>& images() const;
};
} // namespace wmtk::components::adaptive_tessellation::operations::internal