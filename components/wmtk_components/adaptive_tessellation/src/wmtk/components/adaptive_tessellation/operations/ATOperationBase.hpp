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


template <>
struct wmtk::operations::OperationSettings<ATOperationBase>
{
    InvariantCollection invariants;
    void initialize_invariants(const TriMesh& mesh);
};

class ATOperationBase : public tri_mesh::TriMeshOperation, private TupleOperation
{
public:
    ATOperationBase(
        const TriMesh& uv_mesh,
        const TriMesh& position_mesh,
        const std::vector<EdgeMesh>& edge_meshes,
        const std::map<Mesh, Mesh>& edge_meshes_map,
        const Tuple& t,
        const OperationSettings<ATOperationBase>& settings);

protected:
    bool execute() override;
    Tuple m_output_tuple;
    std::map<Mesh, Mesh> m_sibling_meshes_map;
    std::vector<EdgeMesh> m_edge_meshes;
    const TriMesh& m_position_mesh;
};
} // namespace wmtk::components::adaptive_tessellation::operations