#pragma once
#include <optional>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class TetEdgeSplitWithTags;
}

template <>
struct OperationSettings<tet_mesh::TetEdgeSplitWithTags>
{
    MeshAttributeHandle<long> vertex_tag_handle;
    MeshAttributeHandle<long> edge_tag_handle;
    MeshAttributeHandle<double> pos_handle;
    long split_vertex_tag_value;

    InvariantCollection invariants;
    void initialize_invariants(const TetMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tet_mesh {
class TetEdgeSplitWithTags : public TetMeshOperation, private TupleOperation
{
public:
    TetEdgeSplitWithTags(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TetEdgeSplitWithTags>& settings);

    std::string name() const override;

    Tuple new_vertex() const;


    Tuple return_tuple() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<TetEdgeSplitWithTags>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations