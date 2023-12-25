#include "Marching.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>

namespace wmtk::components::internal {

Marching::Marching(
    TriMesh& mesh,
    MeshAttributeHandle<double>& position_handle,
    MeshAttributeHandle<long>& vertex_tag,
    MeshAttributeHandle<long>& edge_tag,
    MeshAttributeHandle<long>& filter_tag,
    const long input_tag_value,
    const long embedding_tag_value,
    const long split_tag_value)
    : m_mesh(mesh)
    , m_position_handle(position_handle)
    , m_vertex_tag(vertex_tag)
    , m_edge_tag(edge_tag)
    , m_filter_tag(filter_tag)
    , m_input_tag_value(input_tag_value)
    , m_embedding_tag_value(embedding_tag_value)
    , m_split_tag_value(split_tag_value)
{}

void Marching::process()
{
    using namespace operations;

    // remove strategies and build new ones
    m_mesh.m_split_strategies.clear();
    m_mesh.m_collapse_strategies.clear();

    m_pos_attribute = std::make_unique<attribute::AttributeInitializationHandle<double>>(
        m_mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3, true));

    m_pos_attribute->trimesh_standard_split_strategy().set_standard_split_rib_strategy(
        operations::NewAttributeStrategy::SplitRibBasicStrategy::Mean);


    Scheduler m_scheduler;

    wmtk::MeshAttributeHandle<long> todo_edgesplit_same_handle =
        m_mesh.register_attribute<long>("todo_edgesplit_same_handle", wmtk::PrimitiveType::Edge, 1);

    wmtk::Accessor<long> acc_vertex_tag = m_mesh.create_accessor(m_vertex_tag);
    wmtk::Accessor<long> acc_edge_tag = m_mesh.create_accessor(m_edge_tag);
    wmtk::Accessor<long> acc_filter = m_mesh.create_accessor(m_filter_tag);
    wmtk::Accessor<double> acc_pos = m_mesh.create_accessor(m_position_handle);
    wmtk::Accessor<long> acc_todo_edgesplit_same_tag =
        m_mesh.create_accessor(todo_edgesplit_same_handle);

    // edge split
    {
        // compute the todo list for the split edge with the same ends
        const std::vector<Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);
        for (const Tuple& edge : edges) {
            long vt0, vt1, ft;
            ft = acc_filter.const_scalar_attribute(edge);
            if (ft == 0) {
                continue;
            }
            vt0 = acc_vertex_tag.scalar_attribute(edge);
            vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
            if ((vt0 == m_input_tag_value && vt1 == m_embedding_tag_value) ||
                (vt1 == m_input_tag_value && vt0 == m_embedding_tag_value)) {
                acc_todo_edgesplit_same_tag.scalar_attribute(edge) = 1;
            }
        }
        //// using scheduler to do edge splitting
        // OperationSettings<tri_mesh::EdgeSplitWithTag> settings_split(m_mesh);
        // settings_split.edge_tag = m_edge_tag;
        // settings_split.vertex_tag = m_vertex_tag;
        // settings_split.embedding_tag_value = m_embedding_tag_value;
        // settings_split.need_embedding_tag_value = true;
        // settings_split.split_at_midpoint_settings.split_boundary_edges = !m_lock_boundary;
        // settings_split.split_at_midpoint_settings.position = m_position_handle;
        // settings_split.split_edge_tag_value = m_embedding_tag_value;
        // settings_split.split_vertex_tag_value = m_split_tag_value;
        // settings_split.split_todo = todo_edgesplit_same_handle;
        //
        // m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>("edge_split", settings_split);
        // while (true) {
        //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");
        //     if (m_scheduler.number_of_successful_operations() == 0) {
        //         break;
        //     }
        // }
    }

    // link the split vertices
    for (const Tuple& t : m_mesh.get_all(PrimitiveType::Edge)) {
        long vt0, vt1;
        vt0 = acc_vertex_tag.const_scalar_attribute(t);
        vt1 = acc_vertex_tag.const_scalar_attribute(m_mesh.switch_vertex(t));
        if (vt0 == m_split_tag_value && vt1 == m_split_tag_value) {
            acc_edge_tag.scalar_attribute(t) = m_split_tag_value;
        }
    }
}

} // namespace wmtk::components::internal
