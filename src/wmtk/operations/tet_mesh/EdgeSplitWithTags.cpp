#include "EdgeSplitWithTags.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::EdgeSplitWithTags>::initialize_invariants(const TetMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
    invariants.add(std::make_shared<TodoInvariant>(m, split_todo_handle));
}

bool OperationSettings<tet_mesh::EdgeSplitWithTags>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants) &&
           find_invariants_in_collection_by_type<TodoInvariant>(invariants);
}

namespace tet_mesh {

EdgeSplitWithTags::EdgeSplitWithTags(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSplitWithTags>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(m_settings.are_invariants_initialized());
}

bool EdgeSplitWithTags::execute()
{
    Accessor<long> acc_vt = mesh().create_accessor(m_settings.vertex_tag_handle);
    Accessor<long> acc_et = mesh().create_accessor(m_settings.edge_tag_handle);
    Accessor<double> acc_pos = mesh().create_accessor(m_settings.pos_handle);
    long et = acc_et.scalar_attribute(input_tuple());
    Eigen::Vector3d p0 = acc_pos.vector_attribute(input_tuple());
    Eigen::Vector3d p1 = acc_pos.vector_attribute(mesh().switch_vertex(input_tuple()));

    auto return_data = mesh().split_edge(input_tuple(), hash_accessor());
    m_output_tuple = return_data.m_output_tuple;
    acc_pos.vector_attribute(mesh().switch_vertex(m_output_tuple)) = (p0 + p1) * 0.5;
    acc_et.scalar_attribute(m_output_tuple) = et;
    acc_et.scalar_attribute(mesh().switch_edge(mesh().switch_face(mesh().switch_tetrahedron(
        mesh().switch_face(mesh().switch_edge(mesh().switch_vertex(m_output_tuple))))))) = et;
    acc_vt.scalar_attribute(mesh().switch_vertex(m_output_tuple)) =
        m_settings.split_vertex_tag_value;

    return true;
}

std::string EdgeSplitWithTags::name() const
{
    return "tet_mesh_split_edge_with_tags";
}

Tuple EdgeSplitWithTags::new_vertex() const
{
    return m_output_tuple;
}

Tuple EdgeSplitWithTags::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeSplitWithTags::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        // TODO
        // return modified_triangles();
    } else if (type == PrimitiveType::Vertex) {
        return {new_vertex()};
    }
    return {};
}
} // namespace tet_mesh
} // namespace wmtk::operations