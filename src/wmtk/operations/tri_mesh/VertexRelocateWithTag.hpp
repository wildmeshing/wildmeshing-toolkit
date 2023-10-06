#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexRelocateWithTag;
}

template <>
struct OperationSettings<tri_mesh::VertexRelocateWithTag>
{
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> vertex_tag;
    MeshAttributeHandle<long> edge_tag;
    double offset_distance;
    int iteration_time_for_optimal_position = 8;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    bool smooth_boundary = false;
    InvariantCollection invariants;
};

namespace tri_mesh {
class VertexRelocateWithTag : public TriMeshOperation, private TupleOperation
{
public:
    VertexRelocateWithTag(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexRelocateWithTag>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;

protected:
    bool before() const override;
    bool execute() override;

private:
    enum { EMBEDDING_CASE, OFFSET_CASE, INPUT_CASE };
    double get_area(const Tuple& t);
    bool is_invert();
    void modify_pos(const Eigen::Vector3d& origin_pos);
    void relocate(const int case_num);
    void push_offset();
    void update_topology();

    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    const OperationSettings<VertexRelocateWithTag>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
