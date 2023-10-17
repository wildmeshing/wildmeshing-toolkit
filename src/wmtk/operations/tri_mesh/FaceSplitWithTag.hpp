#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "FaceSplitAtMidPoint.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplitWithTag;
}

template <>
struct OperationSettings<tri_mesh::FaceSplitWithTag>
{
    OperationSettings<tri_mesh::FaceSplitAtMidPoint> face_split_settings;
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> vertex_tag;
    MeshAttributeHandle<long> edge_tag;
    MeshAttributeHandle<long> split_todo;
    long split_vertex_tag_value;
    long embedding_tag_value;
    bool need_embedding_tag_value;
    void initialize_invariants(const TriMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tri_mesh {
class FaceSplitWithTag : public TriMeshOperation, private TupleOperation
{
public:
    FaceSplitWithTag(Mesh& m, const Tuple& t, const OperationSettings<FaceSplitWithTag>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    Accessor<double> m_pos_accessor;
    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    Accessor<long> m_split_todo_accessor;

    const OperationSettings<FaceSplitWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
