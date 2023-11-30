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
    void initialize_invariants(const Mesh& m, const TriMesh& uv_m);

    // handle to vertex uv coordinates
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

protected:
    Tuple m_output_tuple;
};
} // namespace wmtk::components::adaptive_tessellation::operations
