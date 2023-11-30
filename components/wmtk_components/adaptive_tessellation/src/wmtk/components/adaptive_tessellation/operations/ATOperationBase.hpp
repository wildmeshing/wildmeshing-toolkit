#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include <wmtk/operations/tri_mesh/TriMeshOperation.hpp>

namespace wmtk::components::adaptive_tessellation::operations {

class ATOperationBase;
}


template <>
struct wmtk::operations::OperationSettings<
    wmtk::components::adaptive_tessellation::operations::ATOperationBase>
{
    InvariantCollection invariants;
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior edge
    // op input_m = uv_mesh, boundary edge op input_m = position_mesh)
    // The invariant that is shared among the operations besides the base invariants is the
    // no-triangle-inversion of the uv_mesh
    void initialize_invariants(const Mesh& input_m, const TriMesh& uv_m);

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    MeshAttributeHandle<double> uv;

    std::shared_ptr<TriMesh> uv_mesh_ptr;
    std::shared_ptr<TriMesh> position_mesh_ptr;
    std::vector<std::shared_ptr<EdgeMesh>> edge_mesh_ptrs;
    std::map<Mesh*, Mesh*> sibling_meshes_map;
};
namespace wmtk::components::adaptive_tessellation::operations {

class ATOperationBase : public wmtk::operations::tri_mesh::TriMeshOperation,
                        private wmtk::operations::TupleOperation
{
public:
    template <typename T>
    using OperationSettings = wmtk::operations::OperationSettings<T>;
    ATOperationBase(Mesh& mesh, const Tuple& t, const OperationSettings<ATOperationBase>& settings);

    const TriMesh& uv_mesh() const;
    const TriMesh& position_mesh() const;
    const EdgeMesh& sibling_edge_mesh(const EdgeMesh* my_edge_mesh) const;

protected:
    Tuple m_output_tuple;
};
} // namespace wmtk::components::adaptive_tessellation::operations
