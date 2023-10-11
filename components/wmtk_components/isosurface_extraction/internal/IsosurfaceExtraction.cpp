#include "IsosurfaceExtraction.hpp"

#include <igl/edges.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseRemeshingWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitRemeshingWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapRemeshingWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeTagLinker.hpp>
#include <wmtk/operations/tri_mesh/FaceSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/VertexRelocateWithTag.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialSmooth.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {


void IsosurfaceExtraction::generate_offset_todo_tags(bool isDifferent)
{
    Accessor<long> vertex_tag_accessor = m_mesh.create_accessor(m_vertex_tag_handle);
    Accessor<long> edge_tag_accessor = m_mesh.create_accessor(m_edge_tag_handle);
    Accessor<long> split_todo_accessor = m_mesh.create_accessor(m_split_todo_handle);
    const std::vector<wmtk::Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);

    long itr = 0;
    for (const Tuple& edge : edges) {
        long vt0 = vertex_tag_accessor.const_vector_attribute(edge)(0);
        long vt1 = vertex_tag_accessor.const_vector_attribute(m_mesh.switch_vertex(edge))(0);
        long et = edge_tag_accessor.const_vector_attribute(edge)(0);
        if (isDifferent) {
            if ((vt0 == input_tag_value && vt1 == embedding_tag_value) ||
                (vt1 == input_tag_value && vt0 == embedding_tag_value)) {
                split_todo_accessor.vector_attribute(edge)(0) = 1;
            } else {
                split_todo_accessor.vector_attribute(edge)(0) = 0;
            }
        } else {
            if (vt0 == input_tag_value && vt1 == input_tag_value && et == embedding_tag_value) {
                split_todo_accessor.vector_attribute(edge)(0) = 1;
            } else {
                split_todo_accessor.vector_attribute(edge)(0) = 0;
            }
        }
        itr++;
    }
}

IsosurfaceExtraction::IsosurfaceExtraction(
    TriMesh& mesh,
    const double length,
    const bool lock_boundary,
    long input_tag_value_,
    long embedding_tag_value_,
    long offset_tag_value_,
    double offset_distance_)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_scheduler(m_mesh)
    , input_tag_value(input_tag_value_)
    , embedding_tag_value(embedding_tag_value_)
    , offset_tag_value(offset_tag_value_)
    , offset_distance(offset_distance_)
{
    using namespace operations;

    // for the offset building
    const std::vector<wmtk::Tuple>& edges = m_mesh.get_all(wmtk::PrimitiveType::Edge);
    VectorXl todo_tags_same;
    todo_tags_same.resize(edges.size());
    mesh_utils::set_matrix_attribute(todo_tags_same, "m_split_todo", PrimitiveType::Edge, m_mesh);
    m_mesh.register_attribute<long>("m_split_todo", PrimitiveType::Edge, 1);

    // auto tag_handle = m_mesh.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);
    // auto tag_acc = m_mesh.create_accessor(tag_handle);
    // for (const Tuple& e : edges) {
    //     tag_acc.scalar_attribute(e) = 0;
    // }


    // register the attributes
    m_position_handle = m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    m_vertex_tag_handle = m_mesh.get_attribute_handle<long>("m_vertex_tags", PrimitiveType::Vertex);
    m_edge_tag_handle = m_mesh.get_attribute_handle<long>("m_edge_tags", PrimitiveType::Edge);
    m_split_todo_handle = m_mesh.get_attribute_handle<long>("m_split_todo", PrimitiveType::Edge);

    // offset Computation
    {
        // offset corner case
        OperationSettings<tri_mesh::FaceSplitWithTag> face_split_with_tag;
        face_split_with_tag.position = m_position_handle;
        face_split_with_tag.vertex_tag = m_vertex_tag_handle;
        face_split_with_tag.edge_tag = m_edge_tag_handle;
        face_split_with_tag.input_tag_value = input_tag_value;
        face_split_with_tag.embedding_tag_value = embedding_tag_value;
        face_split_with_tag.offset_tag_value = offset_tag_value;
        m_scheduler.add_operation_type<tri_mesh::FaceSplitWithTag>(
            "face_split_with_tag",
            face_split_with_tag);

        OperationSettings<tri_mesh::EdgeSplitWithTag> split_edge_with_different_tag;
        split_edge_with_different_tag.position = m_position_handle;
        split_edge_with_different_tag.vertex_tag = m_vertex_tag_handle;
        split_edge_with_different_tag.edge_tag = m_edge_tag_handle;
        split_edge_with_different_tag.split_todo = m_split_todo_handle;
        split_edge_with_different_tag.split_when_tags = TAGS_DIFFERENT;
        split_edge_with_different_tag.input_tag_value = input_tag_value;
        split_edge_with_different_tag.embedding_tag_value = embedding_tag_value;
        split_edge_with_different_tag.offset_tag_value = offset_tag_value;
        split_edge_with_different_tag.split_settings.split_boundary_edges = !m_lock_boundary;
        split_edge_with_different_tag.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>(
            "split_edge_with_different_tag_to_build_offset",
            split_edge_with_different_tag);

        OperationSettings<tri_mesh::EdgeSplitWithTag> split_edge_with_same_tag;
        split_edge_with_same_tag.position = m_position_handle;
        split_edge_with_same_tag.vertex_tag = m_vertex_tag_handle;
        split_edge_with_same_tag.edge_tag = m_edge_tag_handle;
        split_edge_with_same_tag.split_todo = m_split_todo_handle;
        split_edge_with_same_tag.split_when_tags = TAGS_SAME;
        split_edge_with_same_tag.input_tag_value = input_tag_value;
        split_edge_with_same_tag.embedding_tag_value = embedding_tag_value;
        split_edge_with_same_tag.offset_tag_value = offset_tag_value;
        split_edge_with_same_tag.split_settings.split_boundary_edges = !m_lock_boundary;
        split_edge_with_same_tag.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>(
            "split_edge_with_same_tag_to_build_offset",
            split_edge_with_same_tag);

        // link offset and update tags
        OperationSettings<tri_mesh::EdgeTagLinker> link_mesh_tag;
        link_mesh_tag.vertex_tag = m_vertex_tag_handle;
        link_mesh_tag.edge_tag = m_edge_tag_handle;
        link_mesh_tag.input_tag_value = input_tag_value;
        link_mesh_tag.embedding_tag_value = embedding_tag_value;
        link_mesh_tag.offset_tag_value = offset_tag_value;
        m_scheduler.add_operation_type<tri_mesh::EdgeTagLinker>("link_mesh_tag", link_mesh_tag);
    }

    // remeshing
    {
        // split
        OperationSettings<tri_mesh::EdgeSplitRemeshingWithTag> split_edge_remeshing_with_tag;
        split_edge_remeshing_with_tag.position = m_position_handle;
        split_edge_remeshing_with_tag.vertex_tag = m_vertex_tag_handle;
        split_edge_remeshing_with_tag.edge_tag = m_edge_tag_handle;
        split_edge_remeshing_with_tag.input_tag_value = input_tag_value;
        split_edge_remeshing_with_tag.embedding_tag_value = embedding_tag_value;
        split_edge_remeshing_with_tag.offset_tag_value = offset_tag_value;
        split_edge_remeshing_with_tag.min_squared_length = m_length_max * m_length_max;
        split_edge_remeshing_with_tag.split_settings.split_boundary_edges = !m_lock_boundary;
        split_edge_remeshing_with_tag.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitRemeshingWithTag>(
            "split_edge_remeshing_with_tag",
            split_edge_remeshing_with_tag);

        // collapse
        OperationSettings<tri_mesh::EdgeCollapseRemeshingWithTag> collapse_edge_remeshing_with_tag;
        collapse_edge_remeshing_with_tag.position = m_position_handle;
        collapse_edge_remeshing_with_tag.vertex_tag = m_vertex_tag_handle;
        collapse_edge_remeshing_with_tag.edge_tag = m_edge_tag_handle;
        collapse_edge_remeshing_with_tag.input_tag_value = input_tag_value;
        collapse_edge_remeshing_with_tag.embedding_tag_value = embedding_tag_value;
        collapse_edge_remeshing_with_tag.offset_tag_value = offset_tag_value;
        collapse_edge_remeshing_with_tag.max_squared_length = m_length_min * m_length_min;
        collapse_edge_remeshing_with_tag.collapse_settings.collapse_boundary_edges =
            !m_lock_boundary;
        collapse_edge_remeshing_with_tag.collapse_towards_boundary = true;
        collapse_edge_remeshing_with_tag.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeCollapseRemeshingWithTag>(
            "collapse_edge_remeshing_with_tag",
            collapse_edge_remeshing_with_tag);

        // swap
        OperationSettings<tri_mesh::EdgeSwapRemeshingWithTag> swap_edge_remeshing_with_tag;
        swap_edge_remeshing_with_tag.position = m_position_handle;
        swap_edge_remeshing_with_tag.vertex_tag = m_vertex_tag_handle;
        swap_edge_remeshing_with_tag.edge_tag = m_edge_tag_handle;
        swap_edge_remeshing_with_tag.input_tag_value = input_tag_value;
        swap_edge_remeshing_with_tag.embedding_tag_value = embedding_tag_value;
        swap_edge_remeshing_with_tag.offset_tag_value = offset_tag_value;
        swap_edge_remeshing_with_tag.must_improve_valence = true;
        m_scheduler.add_operation_type<tri_mesh::EdgeSwapRemeshingWithTag>(
            "swap_edge_remeshing_with_tag",
            swap_edge_remeshing_with_tag);

        // relocate
        OperationSettings<tri_mesh::VertexRelocateWithTag> relocate_vertex_with_tag;
        relocate_vertex_with_tag.position = m_position_handle;
        relocate_vertex_with_tag.vertex_tag = m_vertex_tag_handle;
        relocate_vertex_with_tag.edge_tag = m_edge_tag_handle;
        relocate_vertex_with_tag.iteration_time_for_optimal_position = 8;
        relocate_vertex_with_tag.input_tag_value = input_tag_value;
        relocate_vertex_with_tag.embedding_tag_value = embedding_tag_value;
        relocate_vertex_with_tag.offset_distance = offset_distance;
        relocate_vertex_with_tag.offset_tag_value = offset_tag_value;
        m_scheduler.add_operation_type<tri_mesh::VertexRelocateWithTag>(
            "relocate_vertex_with_tag",
            relocate_vertex_with_tag);
    }
}

void IsosurfaceExtraction::process(const long iteration_times)
{
    // m_scheduler.run_operation_on_all(PrimitiveType::Face, "face_split_with_tag");

    // firstly, we need to create a todo tags to make sure we will do right operations to all edges
    // and vertices in needed.
    generate_offset_todo_tags(false);
    m_scheduler.run_operation_on_all(
        PrimitiveType::Edge,
        "split_edge_with_same_tag_to_build_offset");

    auto acc = m_mesh.create_accessor(m_split_todo_handle);
    for (const Tuple& e : m_mesh.get_all(wmtk::PrimitiveType::Edge)) {
        acc.vector_attribute(e)(0) = 0;
    }

    // generate_offset_todo_tags(true);
    //  m_scheduler.run_operation_on_all(
    //      PrimitiveType::Edge,
    //      "split_edge_with_different_tag_to_build_offset");
    //  m_scheduler.run_operation_on_all(PrimitiveType::Edge, "link_mesh_tag");

    // for (long i = 0; i < iteration_times; ++i) {
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split_edge_remeshing_with_tag");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge,
    //     "collapse_edge_remeshing_with_tag");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap_edge_remeshing_with_tag");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "relocate_vertex_with_tag");
    // }
}

} // namespace wmtk::components::internal