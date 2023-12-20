#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "FaceSplitAtMidPoint.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplitWithTag;
}

template <>
struct OperationSettings<tri_mesh::FaceSplitWithTag> : public OperationSettingsBase
{
    OperationSettings<tri_mesh::FaceSplitWithTag>(TriMesh& m)
        : m_mesh(m)
        , face_split_settings(m)
    {}

    TriMesh& m_mesh;

    OperationSettings<tri_mesh::FaceSplitAtMidPoint> face_split_settings;
    MeshAttributeHandle<long> vertex_tag;
    MeshAttributeHandle<long> edge_tag;
    MeshAttributeHandle<long> split_todo;
    long split_vertex_tag_value;
    long embedding_tag_value;
    bool need_embedding_tag_value;

    void create_invariants();
};

namespace tri_mesh {
class FaceSplitWithTag : public TriMeshOperation, private TupleOperation
{
public:
    FaceSplitWithTag(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<FaceSplitWithTag>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Face; }
    std::vector<Simplex> modified_primitives() const override;

    std::vector<Simplex> unmodified_primitives() const override;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    Accessor<long> m_split_todo_accessor;

    const OperationSettings<FaceSplitWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
