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
    void initialize_invariants(const TriMesh& mesh);

    std::shared_ptr<TriMesh> position_mesh;
    std::vector<std::shared_ptr<EdgeMesh>> edge_meshes;
    std::map<Mesh*, Mesh*> sibling_meshes_map;
};
namespace wmtk::components::adaptive_tessellation::operations {

class ATOperationBase : public wmtk::operations::tri_mesh::TriMeshOperation,
                        private wmtk::operations::TupleOperation
{
public:
    template <typename T>
    using OperationSettings = wmtk::operations::OperationSettings<T>;
    ATOperationBase(
        TriMesh& uv_mesh,
        const Tuple& t,
        const OperationSettings<ATOperationBase>& settings);

protected:
    bool execute() override;
    Tuple m_output_tuple;
};
} // namespace wmtk::components::adaptive_tessellation::operations
