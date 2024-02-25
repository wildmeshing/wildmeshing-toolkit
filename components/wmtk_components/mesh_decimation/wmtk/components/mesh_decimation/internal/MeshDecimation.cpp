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

namespace wmtk::components::internal {

MeshDecimation::MeshDecimation(
    Mesh& mesh,
    std::string constriant_name,
    int64_t constrait_value,
    double target_len,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_constriant_name(constriant_name)
    , m_constrait_value(constrait_value)
    , m_target_len(target_len)
    , m_pass_through_attributes(pass_through_attributes)
{}

void MeshDecimation::process()
{
    using namespace wmtk::attribute;
    using namespace wmtk::invariants;
    PrimitiveType PV = PrimitiveType::Vertex;
    PrimitiveType PE = PrimitiveType::Edge;
    PrimitiveType PF = PrimitiveType::Triangle;
    PrimitiveType PT = PrimitiveType::Tetrahedron;

    volatile PrimitiveType x = m_mesh.top_simplex_type();

    MeshAttributeHandle cell_tag_handle =
        m_mesh.get_attribute_handle<int64_t>(m_constriant_name, m_mesh.top_simplex_type());
    MeshAttributeHandle position = m_mesh.get_attribute_handle<double>("vertices", PV);
    MeshAttributeHandle edge_handle = m_mesh.register_attribute<int64_t>("todo_edge_", PE, 1);
    MeshAttributeHandle vertex_handle = m_mesh.register_attribute<int64_t>("todo_vertex_", PV, 1);
    MeshAttributeHandle edge_len_handle = m_mesh.register_attribute<double>("len_edge_", PE, 1);

    Accessor<int64_t> acc_cell = m_mesh.create_accessor<int64_t>(cell_tag_handle);
    Accessor<double> acc_pos = m_mesh.create_accessor<double>(position);
    Accessor<int64_t> acc_edge = m_mesh.create_accessor<int64_t>(edge_handle);
    Accessor<int64_t> acc_vertex = m_mesh.create_accessor<int64_t>(vertex_handle);
    Accessor<double> acc_len = m_mesh.create_accessor<double>(edge_len_handle);

    switch (m_mesh.top_cell_dimension()) {
    case 2:
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
    case 3: {
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
    default:
        std::runtime_error("MeshDecimation.cpp: mesh_decimation component only supports tetmesh "
                           "and trimesh for now!");
    }

    for (const Tuple& edge : m_mesh.get_all(PE)) {
        if (acc_vertex.scalar_attribute(edge) == 1 ||
            acc_vertex.scalar_attribute(m_mesh.switch_tuple(edge, PV)) == 1) {
            acc_edge.scalar_attribute(edge) = 1;
        }
        acc_len.scalar_attribute(edge) = (acc_pos.vector_attribute(edge) -
                                          acc_pos.vector_attribute(m_mesh.switch_tuple(edge, PV)))
                                             .norm();
    }

    auto op_scaffold = std::make_shared<operations::EdgeCollapse>(m_mesh);

    op_scaffold->add_invariant(
        std::make_shared<TodoInvariant>(m_mesh, edge_handle.as<int64_t>(), 0));
    op_scaffold->add_invariant(std::make_shared<TodoSmallerInvariant>(
        m_mesh,
        edge_len_handle.as<double>(),
        4.0 / 5.0 * m_target_len));

    auto m_amips = std::make_shared<function::AMIPS>(m_mesh, position);
    auto m_link_conditions = std::make_shared<InvariantCollection>(m_mesh);
    m_link_conditions->add(std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
    auto m_function_invariant =
        std::make_shared<SmallerFunctionInvariant>(m_mesh.top_simplex_type(), m_amips, 30);
    auto m_inversion_invariant =
        std::make_shared<SimplexInversionInvariant>(m_mesh, position.as<double>());

    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto m_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_len_handle,
            position,
            compute_edge_length);
    auto m_prio_short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        auto acc = m_mesh.create_accessor<double>(edge_len_handle);
        return std::vector<double>({acc.scalar_attribute(s.tuple())});
    };


    op_scaffold->add_invariant(m_link_conditions);
    op_scaffold->add_invariant(m_function_invariant);
    op_scaffold->add_invariant(m_inversion_invariant);

    op_scaffold->add_transfer_strategy(m_edge_length_update);

    op_scaffold->set_priority(m_prio_short_edges_first);

    op_scaffold->set_new_attribute_strategy(
        position,
        wmtk::operations::CollapseBasicStrategy::Mean);
    op_scaffold->set_new_attribute_strategy(vertex_handle);

    op_scaffold->set_new_attribute_strategy(edge_handle);
    op_scaffold->set_new_attribute_strategy(edge_len_handle);
    op_scaffold->set_new_attribute_strategy(cell_tag_handle);

    // pass_through
    for (const auto& attr : m_pass_through_attributes) {
        op_scaffold->set_new_attribute_strategy(attr);
    }

    while (true) {
        Scheduler scheduler;
        SchedulerStats pass_stats = scheduler.run_operation_on_all(*op_scaffold);
        if (pass_stats.number_of_successful_operations() == 0) break;
    }
}

} // namespace wmtk::components::internal
