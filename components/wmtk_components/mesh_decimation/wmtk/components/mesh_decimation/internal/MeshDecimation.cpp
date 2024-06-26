#include "MeshDecimation.hpp"
#include "wmtk/Scheduler.hpp"
#include "wmtk/function/simplex/AMIPS.hpp"
#include "wmtk/invariants/MultiMeshLinkConditionInvariant.hpp"
#include "wmtk/invariants/SimplexInversionInvariant.hpp"
#include "wmtk/invariants/SmallerFunctionInvariant.hpp"
#include "wmtk/invariants/TodoInvariant.hpp"
#include "wmtk/operations/EdgeCollapse.hpp"
#include "wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp"
#include "wmtk/operations/attribute_update/AttributeTransferStrategy.hpp"
#include "wmtk/utils/Logger.hpp"

namespace wmtk::components::internal {

MeshDecimation::MeshDecimation(
    Mesh& mesh,
    attribute::MeshAttributeHandle constrainted_cell_tag_handle,
    double target_len,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_constrainted_cell_tag_handle(constrainted_cell_tag_handle)
    , m_target_len(target_len)
    , m_pass_through_attributes(pass_through_attributes)
{}

void MeshDecimation::process()
{
    using namespace wmtk::attribute;
    using namespace wmtk::invariants;
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    MeshAttributeHandle& cell_tag_handle = m_constrainted_cell_tag_handle;
    MeshAttributeHandle position = m_mesh.get_attribute_handle<double>("vertices", PV);
    MeshAttributeHandle todo_edge_handle =
        m_mesh.register_attribute<int64_t>("mesh_decimation_todo_edge", PE, 1);
    MeshAttributeHandle todo_vertex_handle =
        m_mesh.register_attribute<int64_t>("mesh_decimation_todo_vertex", PV, 1);
    MeshAttributeHandle edge_len_handle =
        m_mesh.register_attribute<double>("mesh_decimation_edge_len", PE, 1);

    Accessor<int64_t> acc_cell = m_mesh.create_accessor<int64_t>(cell_tag_handle);
    Accessor<double> acc_pos = m_mesh.create_accessor<double>(position);
    Accessor<int64_t> acc_edge = m_mesh.create_accessor<int64_t>(todo_edge_handle);
    Accessor<int64_t> acc_vertex = m_mesh.create_accessor<int64_t>(todo_vertex_handle);
    Accessor<double> acc_len = m_mesh.create_accessor<double>(edge_len_handle);

    // build edge tag to protect to topology of the tagged input
    switch (m_mesh.top_simplex_type()) {
    case PF:
        for (const Tuple& edge : m_mesh.get_all(PE)) {
            if (m_mesh.is_boundary(PE, edge)) {
                acc_vertex.scalar_attribute(edge) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) = 1;

                acc_edge.scalar_attribute(edge) = 1;
            } else if (
                acc_cell.scalar_attribute(edge) !=
                acc_cell.scalar_attribute(m_mesh.switch_tuple(edge, PF))) {
                acc_vertex.scalar_attribute(edge) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) = 1;

                acc_edge.scalar_attribute(edge) = 1;
            }
        }
        break;
    case PT: {
        for (const Tuple& face : m_mesh.get_all(PF)) {
            if (m_mesh.is_boundary(PF, face)) {
                acc_vertex.scalar_attribute(face) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuple(face, PV)) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuples(face, {PE, PV})) = 1;

                acc_edge.scalar_attribute(face) = 1;
                acc_edge.scalar_attribute(m_mesh.switch_tuple(face, PE)) = 1;
                acc_edge.scalar_attribute(m_mesh.switch_tuples(face, {PV, PE})) = 1;
            } else if (
                acc_cell.scalar_attribute(face) !=
                acc_cell.scalar_attribute(m_mesh.switch_tuple(face, PT))) {
                acc_vertex.scalar_attribute(face) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuple(face, PV)) = 1;
                acc_vertex.scalar_attribute(m_mesh.switch_tuples(face, {PE, PV})) = 1;

                acc_edge.scalar_attribute(face) = 1;
                acc_edge.scalar_attribute(m_mesh.switch_tuple(face, PE)) = 1;
                acc_edge.scalar_attribute(m_mesh.switch_tuples(face, {PV, PE})) = 1;
            }
        }
        break;
    }
    case PE:
    case PV:
    default:
        log_and_throw_error("MeshDecimation.cpp: mesh_decimation component only supports tetmesh "
                            "and trimesh for now!");
    }

    // stage 1: tag edges incident to the surface
    for (const Tuple& edge : m_mesh.get_all(PE)) {
        if (acc_vertex.scalar_attribute(edge) == 1 ||
            acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) == 1) {
            acc_edge.scalar_attribute(edge) = 1;
        }
    }
    // stage 2: tag vertices incident to the tagged edge
    for (const Tuple& edge : m_mesh.get_all(PE)) {
        if (acc_edge.scalar_attribute(edge) == 1) {
            acc_vertex.scalar_attribute(edge) = 1;
            acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) = 1;
        }
    }
    // stage 3: tag edges incident to the tagged vertices
    for (const Tuple& edge : m_mesh.get_all(PE)) {
        if (acc_vertex.scalar_attribute(edge) == 1 ||
            acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) == 1) {
            acc_edge.scalar_attribute(edge) = 1;
        }
    }

    // Storing edge lengths and Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_len_handle,
            position,
            compute_edge_length);
    edge_length_update->run_on_all();

    auto m_prio_short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        auto acc = m_mesh.create_accessor<double>(edge_len_handle);
        return std::vector<double>({acc.scalar_attribute(s.tuple())});
    };

    auto op_collapse = std::make_shared<operations::EdgeCollapse>(m_mesh);

    op_collapse->add_invariant(
        std::make_shared<TodoInvariant>(m_mesh, todo_edge_handle.as<int64_t>(), 0));
    op_collapse->add_invariant(
        std::make_shared<TodoSmallerInvariant>(m_mesh, edge_len_handle.as<double>(), m_target_len));

    auto m_amips = std::make_shared<function::AMIPS>(m_mesh, position);
    auto m_link_conditions = std::make_shared<InvariantCollection>(m_mesh);
    m_link_conditions->add(std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
    auto m_function_invariant =
        std::make_shared<SmallerFunctionInvariant>(m_mesh.top_simplex_type(), m_amips, 30);
    auto m_inversion_invariant =
        std::make_shared<SimplexInversionInvariant>(m_mesh, position.as<double>());

    op_collapse->add_invariant(m_link_conditions);
    op_collapse->add_invariant(m_inversion_invariant);
    op_collapse->add_invariant(m_function_invariant);

    op_collapse->add_transfer_strategy(edge_length_update);

    op_collapse->set_priority(m_prio_short_edges_first);

    op_collapse->set_new_attribute_strategy(
        position,
        wmtk::operations::CollapseBasicStrategy::Mean);
    op_collapse->set_new_attribute_strategy(todo_vertex_handle);

    op_collapse->set_new_attribute_strategy(todo_edge_handle);
    op_collapse->set_new_attribute_strategy(edge_len_handle);
    op_collapse->set_new_attribute_strategy(cell_tag_handle);

    // pass_through
    for (const auto& attr : m_pass_through_attributes) {
        op_collapse->set_new_attribute_strategy(attr);
    }

    while (true) {
        Scheduler scheduler;
        SchedulerStats pass_stats = scheduler.run_operation_on_all(*op_collapse);
        m_mesh.consolidate();
        if (pass_stats.number_of_successful_operations() == 0) {
            break;
        }
    }
}

} // namespace wmtk::components::internal
