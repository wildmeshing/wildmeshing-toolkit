#pragma once
#include <optional>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"
#include "TetSplit.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class TetSplitWithTags;
} // namespace tet_mesh


template <>
struct OperationSettings<tet_mesh::TetSplitWithTags> : public OperationSettingsBase
{
    OperationSettings<tet_mesh::TetSplitWithTags>(TetMesh& m)
        : m_mesh(m)
        , split_settings(m)
    {}

    TetMesh& m_mesh;

    OperationSettings<tet_mesh::TetSplit> split_settings;

    MeshAttributeHandle<long> split_tet_todo_handle;
    MeshAttributeHandle<long> vertex_tag_handle;
    MeshAttributeHandle<long> edge_tag_handle;
    MeshAttributeHandle<double> pos_handle;
    long split_vertex_tag_value;

    void create_invariants();
};

namespace tet_mesh {
class TetSplitWithTags : public TetMeshOperation, private TupleOperation
{
public:
    TetSplitWithTags(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<TetSplitWithTags>& settings);

    std::string name() const override;

    Tuple new_vertex() const;


    Tuple return_tuple() const;
    std::vector<Simplex> modified_primitives() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Tetrahedron; }

    using TetMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<TetSplitWithTags>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations
