#include "RegularSpace.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/FaceSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>

namespace wmtk::components::internal {

RegularSpace::RegularSpace(
    TriMesh& mesh,
    MeshAttributeHandle<double>& position_handle,
    MeshAttributeHandle<long>& vertex_tag,
    MeshAttributeHandle<long>& edge_tag,
    const long input_tag_value,
    const long embedding_tag_value,
    const long split_tag_value,
    const int dimension,
    const bool lock_boundary)
    : m_mesh(mesh)
    , m_position_handle(position_handle)
    , m_vertex_tag(vertex_tag)
    , m_edge_tag(edge_tag)
    , m_input_tag_value(input_tag_value)
    , m_embedding_tag_value(embedding_tag_value)
    , m_split_tag_value(split_tag_value)
    , m_dimension(dimension)
    // , m_lock_boundary(lock_boundary)
    , m_scheduler(mesh)
{}

void RegularSpace::process()
{
    switch (m_dimension) {
    case 0: process_in_0d(); break;
    case 1: process_in_1d(); break;
    case 2: process_in_2d(); break;
    default: throw std::runtime_error("settings went wrong!"); break;
    }
}

void RegularSpace::process_in_0d()
{
    using namespace operations;

    wmtk::MeshAttributeHandle<long> todo_edgesplit_same_handle = m_mesh.register_attribute<long>(
        std::string("todo_edgesplit_same_tag"),
        wmtk::PrimitiveType::Edge,
        1);
    wmtk::Accessor<long> acc_vertex_tag = m_mesh.create_accessor(m_vertex_tag);
    wmtk::Accessor<double> acc_pos = m_mesh.create_accessor(m_position_handle);
    wmtk::Accessor<long> acc_todo_edgesplit_same_tag =
        m_mesh.create_accessor(todo_edgesplit_same_handle);

    // edge split
    {
        // compute the todo list for the split edge with the same ends
        const std::vector<Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);
        for (const Tuple& edge : edges) {
            long vt0, vt1;
            vt0 = acc_vertex_tag.scalar_attribute(edge);
            vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
            if (vt0 == m_input_tag_value && vt1 == m_input_tag_value) {
                acc_todo_edgesplit_same_tag.scalar_attribute(edge) = 1;
            }
        }
        // using scheduler to do edge splitting
        OperationSettings<tri_mesh::EdgeSplitWithTag> settings_split_same;
        settings_split_same.edge_tag = m_edge_tag;
        settings_split_same.vertex_tag = m_vertex_tag;
        settings_split_same.embedding_tag_value = m_embedding_tag_value;
        settings_split_same.need_embedding_tag_value = true;
        // settings_split_same.position = m_position_handle;
        settings_split_same.split_with_tag_settings.split_settings.split_boundary_edges = true;
        settings_split_same.split_with_tag_settings.position = m_position_handle;
        settings_split_same.split_edge_tag_value = m_embedding_tag_value;
        settings_split_same.split_vertex_tag_value = m_split_tag_value;
        settings_split_same.split_todo = todo_edgesplit_same_handle;
        settings_split_same.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>(
            "edge_split",
            settings_split_same);
        while (true) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");
            if (m_scheduler.number_of_successful_operations() == 0) {
                break;
            }
        }
    }
}

void RegularSpace::process_in_1d()
{
    using namespace operations;

    wmtk::MeshAttributeHandle<long> todo_facesplit_handle = m_mesh.register_attribute<long>(
        std::string("todo_facesplit_tag"),
        wmtk::PrimitiveType::Face,
        1);
    wmtk::MeshAttributeHandle<long> todo_edgesplit_same_handle = m_mesh.register_attribute<long>(
        std::string("todo_edgesplit_same_tag"),
        wmtk::PrimitiveType::Edge,
        1);
    // wmtk::MeshAttributeHandle<long> todo_edgesplit_diff_handle = m_mesh.register_attribute<long>(
    //     std::string("todo_edgesplit_diff_tag"),
    //     wmtk::PrimitiveType::Edge,
    //     1);
    wmtk::Accessor<long> acc_vertex_tag = m_mesh.create_accessor(m_vertex_tag);
    wmtk::Accessor<long> acc_edge_tag = m_mesh.create_accessor(m_edge_tag);
    wmtk::Accessor<double> acc_pos = m_mesh.create_accessor(m_position_handle);
    wmtk::Accessor<long> acc_todo_face_tag = m_mesh.create_accessor(todo_facesplit_handle);
    wmtk::Accessor<long> acc_todo_edgesplit_same_tag =
        m_mesh.create_accessor(todo_edgesplit_same_handle);
    // wmtk::Accessor<long> acc_todo_edgesplit_diff_tag =
    //     m_mesh.create_accessor(todo_edgesplit_diff_handle);

    // face split
    {
        const std::vector<Tuple>& faces = m_mesh.get_all(wmtk::PrimitiveType::Face);
        for (const Tuple& face : faces) {
            long vt0, vt1, vt2, et0, et1, et2;
            vt0 = acc_vertex_tag.scalar_attribute(face);
            vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(face));
            vt2 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(face)));
            et0 = acc_edge_tag.scalar_attribute(face);
            et1 = acc_edge_tag.scalar_attribute(m_mesh.switch_edge(face));
            et2 = acc_edge_tag.scalar_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(face)));
            if (vt0 == m_input_tag_value && vt1 == m_input_tag_value && vt2 == m_input_tag_value &&
                et0 == m_input_tag_value && et1 == m_input_tag_value && et2 == m_input_tag_value) {
                acc_todo_face_tag.scalar_attribute(face) = 1;
            }
        }
        // using scheduler to do face splitting
        OperationSettings<tri_mesh::FaceSplitWithTag> settings_split_face;
        settings_split_face.edge_tag = m_edge_tag;
        settings_split_face.embedding_tag_value = m_embedding_tag_value;
        settings_split_face.need_embedding_tag_value = true;
        settings_split_face.face_split_settings.position = m_position_handle;
        settings_split_face.split_todo = todo_facesplit_handle;
        settings_split_face.split_vertex_tag_value = m_split_tag_value;
        settings_split_face.vertex_tag = m_vertex_tag;
        settings_split_face.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::FaceSplitWithTag>(
            "facesplit",
            settings_split_face);
        while (true) {
            m_scheduler.run_operation_on_all(PrimitiveType::Face, "facesplit");
            if (m_scheduler.number_of_successful_operations() == 0) {
                break;
            }
        }
        m_scheduler.run_operation_on_all(PrimitiveType::Face, "facesplit");
    }

    // edge split
    {
        // compute the todo list for the split edge with the same ends
        const std::vector<Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);
        for (const Tuple& edge : edges) {
            long vt0, vt1, et0;
            vt0 = acc_vertex_tag.scalar_attribute(edge);
            vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
            et0 = acc_edge_tag.scalar_attribute(edge);
            if (vt0 == m_input_tag_value && vt1 == m_input_tag_value &&
                et0 == m_embedding_tag_value) {
                acc_todo_edgesplit_same_tag.scalar_attribute(edge) = 1;
            }
        }
        // using scheduler to do edge splitting
        OperationSettings<tri_mesh::EdgeSplitWithTag> settings_split_same;
        settings_split_same.edge_tag = m_edge_tag;
        settings_split_same.vertex_tag = m_vertex_tag;
        settings_split_same.embedding_tag_value = m_embedding_tag_value;
        settings_split_same.need_embedding_tag_value = true;
        // settings_split_same.position = m_position_handle;
        settings_split_same.split_with_tag_settings.split_settings.split_boundary_edges = true;
        settings_split_same.split_with_tag_settings.position = m_position_handle;
        settings_split_same.split_edge_tag_value = m_embedding_tag_value;
        settings_split_same.split_vertex_tag_value = m_split_tag_value;
        settings_split_same.split_todo = todo_edgesplit_same_handle;
        settings_split_same.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>(
            "edge_split",
            settings_split_same);
        while (true) {
            m_scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");
            if (m_scheduler.number_of_successful_operations() == 0) {
                break;
            }
        }
    }

    // compute the todo list for the split edge with the different ends
    // {
    //     const std::vector<Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);
    //     for (const Tuple& edge : edges) {
    //         long vt0, vt1;
    //         vt0 = acc_vertex_tag.scalar_attribute(edge);
    //         vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
    //         if ((vt0 == m_input_tag_value && vt1 == m_embedding_tag_value) ||
    //             (vt1 == m_input_tag_value && vt0 == m_embedding_tag_value)) {
    //             acc_todo_edgesplit_same_tag.scalar_attribute(edge) = 1;
    //         }
    //     }
    // }
    // using scheduler to do edge splitting
    // ...
}

void RegularSpace::process_in_2d()
{
    throw std::runtime_error("2 dimension has not been implemented!");
}

} // namespace wmtk::components::internal
