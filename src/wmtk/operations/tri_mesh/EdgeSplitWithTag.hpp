#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSplitWithTag;
}

enum { TAGS_DIFFERENT, TAGS_SAME };

template <>
struct OperationSettings<tri_mesh::EdgeSplitWithTag>
{
    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> tag;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    bool split_when_tags; // true for same, false for different
    // too short edges get ignored
    double min_squared_length = -1;

    void initialize_invariants(const TriMesh& m);

    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tri_mesh {
class EdgeSplitWithTag : public TriMeshOperation, private TupleOperation
{
public:
    EdgeSplitWithTag(Mesh& m, const Tuple& t, const OperationSettings<EdgeSplitWithTag>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_tag_accessor;

    const OperationSettings<EdgeSplitWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
