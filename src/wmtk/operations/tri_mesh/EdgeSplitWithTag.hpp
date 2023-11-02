#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplitAtMidpoint.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSplitWithTag;
}

enum { TAGS_DIFFERENT, TAGS_SAME };

template <>
struct OperationSettings<tri_mesh::EdgeSplitWithTag>
{
    OperationSettings<tri_mesh::EdgeSplitAtMidpoint> split_with_tag_settings;
    InvariantCollection invariants;
    // handle to vertex position
    // MeshAttributeHandle<double> position;
    // handle to vertex attribute
    MeshAttributeHandle<long> vertex_tag;
    // handle to edge attribute
    MeshAttributeHandle<long> edge_tag;

    // a todo-list attribute, only do splitting when split_todo tag is 1
    MeshAttributeHandle<long> split_todo;

    //      /\        /|\ .
    //     /  \      / 3 \ .
    //    / f  \    /  |f \ .
    //   X- - - > v0-0-X-2->v1
    //    \    /    \  |  /
    //     \  /      \ 1 /
    //      \/        \|/
    // after splitting, the new edges' attributes should be tagged as split_vertex_tag_value
    // edges 1 and 3 will be tagged as the split_edge_tag_value
    // the new vertex X will be tagged as the split_vertex_tag_value
    // the edges 0 and 2's attribute after splitting depends on the need_embedding_tag_value
    // if need_embedding_tag_value is true, 0 and 2's attribute will be the embedding_tag_value,
    // otherwise, their attributes will same to their old neighbour's attribute
    // edge0's = v0's and edge2's = v1's
    long split_vertex_tag_value;
    long split_edge_tag_value;
    long embedding_tag_value;
    bool need_embedding_tag_value;

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
    bool execute() override;

private:
    Tuple m_output_tuple;
    // Accessor<double> m_pos_accessor;
    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    Accessor<long> m_split_todo_accessor;

    const OperationSettings<EdgeSplitWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations