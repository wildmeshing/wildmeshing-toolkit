#include "TetSplitWithTags.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include "TetSplit.hpp"

namespace wmtk::operations {

void OperationSettings<tet_mesh::TetSplitWithTags>::create_invariants()
{
    split_settings.create_invariants();

    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<TodoInvariant>(m_mesh, split_tet_todo_handle));
}

namespace tet_mesh {
TetSplitWithTags::TetSplitWithTags(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<TetSplitWithTags>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Tetrahedron);
}

// TetEdgeSplit::~TetEdgeSplit() = default;

bool TetSplitWithTags::execute()
{
    Accessor<long> acc_vt = mesh().create_accessor(m_settings.vertex_tag_handle);
    Accessor<long> acc_et = mesh().create_accessor(m_settings.edge_tag_handle);
    Accessor<long> acc_tt = mesh().create_accessor(m_settings.split_tet_todo_handle);
    Accessor<double> acc_pos = mesh().create_accessor(m_settings.pos_handle);
    Eigen::Vector3d p0 = acc_pos.vector_attribute(input_tuple());
    Eigen::Vector3d p1 = acc_pos.vector_attribute(mesh().switch_vertex(input_tuple()));
    Eigen::Vector3d p2 =
        acc_pos.vector_attribute(mesh().switch_vertex(mesh().switch_edge(input_tuple())));
    Eigen::Vector3d p3 = acc_pos.vector_attribute(
        mesh().switch_vertex(mesh().switch_edge(mesh().switch_face(input_tuple()))));
    long et = acc_et.scalar_attribute(input_tuple());
    std::optional<long> opposite_tt;
    if (!mesh().is_boundary(input_tuple())) {
        opposite_tt = acc_tt.scalar_attribute(mesh().switch_tetrahedron(input_tuple()));
    }

    TetSplit op(mesh(), input_simplex(), m_settings.split_settings);
    if (!op()) {
        return false;
    }

    m_output_tuple = op.return_tuple();
    acc_pos.vector_attribute(mesh().switch_vertex(m_output_tuple)) = (p0 + p1 + p2 + p3) * 0.25;
    acc_vt.scalar_attribute(mesh().switch_vertex(m_output_tuple)) =
        m_settings.split_vertex_tag_value;
    acc_et.scalar_attribute(mesh().switch_edge(m_output_tuple)) = et;
    if (opposite_tt.has_value()) {
        acc_tt.scalar_attribute(mesh().switch_tetrahedron(mesh().switch_face(
            mesh().switch_edge(mesh().switch_tetrahedron(m_output_tuple))))) = opposite_tt.value();
    }

    return true;
}

std::string TetSplitWithTags::name() const
{
    return "tet_mesh_split_tet_with_tags";
}

Tuple TetSplitWithTags::new_vertex() const
{
    return m_output_tuple;
}

Tuple TetSplitWithTags::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> TetSplitWithTags::modified_primitives(PrimitiveType type) const
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
