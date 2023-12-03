#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>

namespace wmtk::components::adaptive_tessellation::operations::internal {

class ATOperation
{
    std::shared_ptr<TriMesh> m_uv_mesh_ptr;
    std::shared_ptr<TriMesh> m_position_mesh_ptr;
    std::vector<std::shared_ptr<EdgeMesh>> m_edge_mesh_ptrs;
    std::map<EdgeMesh*, EdgeMesh*> m_sibling_meshes_map;

public:
    InvariantCollection invariants;
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior edge
    // op input_m = uv_mesh, boundary edge op input_m = position_mesh)
    // The invariant that is shared among the operations besides the base invariants is the
    // no-triangle-inversion of the uv_mesh
    void initialize_invariants();

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    MeshAttributeHandle<double> m_uv_handle;
    MeshAttributeHandle<double> m_position_handle;
    // Scheduler m_scheduler;

    ATOperation(
        TriMesh& uv_mesh,
        TriMesh& position_mesh,
        std::vector<std::shared_ptr<EdgeMesh>> edge_mesh_ptrs,
        std::map<EdgeMesh*, EdgeMesh*> sibling_meshes_map,
        MeshAttributeHandle<double>& uv_handle,
        MeshAttributeHandle<double>& position_handle);

    const std::shared_ptr<TriMesh> uv_mesh_ptr() const;
    const std::shared_ptr<TriMesh> position_mesh_ptr() const;
    const std::shared_ptr<EdgeMesh> edge_mesh_ptr(long i) const;
    EdgeMesh* sibling_edge_mesh_raw_ptr(EdgeMesh* my_edge_mesh_ptr);
    Simplex sibling_edge(EdgeMesh* my_edge_mesh_ptr, const Simplex& s);
};
} // namespace wmtk::components::adaptive_tessellation::operations::internal
