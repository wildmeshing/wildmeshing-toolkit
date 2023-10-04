
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSwapRemeshingWithTag;
}

template <>
struct OperationSettings<tri_mesh::EdgeSwapRemeshingWithTag>
{
    bool must_improve_valence = false;
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> vertex_tag;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    InvariantCollection invariants;
};

namespace tri_mesh {
class EdgeSwapRemeshingWithTag : public TriMeshOperation, private TupleOperation
{
public:
    EdgeSwapRemeshingWithTag(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<EdgeSwapRemeshingWithTag>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<EdgeSwapRemeshingWithTag>& m_settings;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_vertex_tag_accessor;
};

} // namespace tri_mesh
} // namespace wmtk::operations
