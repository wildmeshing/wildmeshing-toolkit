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
    TriMesh m_uv_mesh;
    TriMesh m_position_mesh;
    std::vector<std::shared_ptr<EdgeMesh>> m_edge_mesh_ptrs;
    std::map<Mesh*, Mesh*> m_sibling_meshes_map;

public:
    InvariantCollection invariants;
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior edge
    // op input_m = uv_mesh, boundary edge op input_m = position_mesh)
    // The invariant that is shared among the operations besides the base invariants is the
    // no-triangle-inversion of the uv_mesh
    void initialize_invariants(const Mesh& input_m, const TriMesh& uv_m);

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    MeshAttributeHandle<double> uv_handle;
    // Scheduler m_scheduler;

    ATOperation(
        TriMesh& uv_mesh,
        TriMesh& position_mesh,
        std::vector<std::shared_ptr<EdgeMesh>> edge_mesh_ptrs,
        std::map<Mesh*, Mesh*> sibling_meshes_map);

    const TriMesh& uv_mesh() const;
    const TriMesh& position_mesh() const;
    const EdgeMesh& edge_mesh(long i) const;
    Mesh* sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr);
    Simplex sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s);
};
} // namespace wmtk::components::adaptive_tessellation::operations::internal