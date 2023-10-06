
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplitWithTag;
}

template <>
struct OperationSettings<tri_mesh::FaceSplitWithTag>
{
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> vertex_tag;
    MeshAttributeHandle<long> edge_tag;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    InvariantCollection invariants;
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
    bool before() const override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    const OperationSettings<FaceSplitWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
