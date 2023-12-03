#include "VertexMoveToTargetPos.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/utils/HelperFunctions.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::VertexMoveToTargetPos>::initialize_invariants(const TetMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(std::make_unique<TodoInvariant>(m, todo_tag_handle));
} // namespace wmtk::operations

namespace tet_mesh {
VertexMoveToTargetPos::VertexMoveToTargetPos(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexMoveToTargetPos>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_pos_accessor(m.create_accessor<double>(settings.position))
    , m_settings{settings}
{}

std::string VertexMoveToTargetPos::name() const
{
    return "tet_mesh_vertex_move_target_pos";
}


bool VertexMoveToTargetPos::execute()
{
    Accessor<double> original_p_acc = mesh().create_accessor<double>(m_settings.position);
    Accessor<double> move_vec_acc = mesh().create_accessor<double>(m_settings.move_vector);
    Eigen::Vector3d original_p = original_p_acc.vector_attribute(input_tuple());
    Eigen::Vector3d move_vec_p = move_vec_acc.vector_attribute(input_tuple());
    Eigen::Vector3d last_best_p = original_p;
    Eigen::Vector3d target_p = original_p + m_settings.alpha * move_vec_p;

    operations::utils::optimize_position(
        mesh(),
        m_settings.position,
        input_tuple(),
        target_p,
        last_best_p,
        PrimitiveType::Face);

    return tet_mesh::VertexAttributesUpdateBase::execute();
}

} // namespace tet_mesh
} // namespace wmtk::operations
