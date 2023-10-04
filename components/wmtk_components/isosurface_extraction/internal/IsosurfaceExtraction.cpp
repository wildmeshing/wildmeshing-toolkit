#include "IsosurfaceExtraction.hpp"

#include <igl/edges.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseRemeshingWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitRemeshingWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialSmooth.hpp>

namespace wmtk::components::internal {

IsosurfaceExtraction::IsosurfaceExtraction(
    TriMesh& mesh,
    const double length,
    const bool lock_boundary,
    long input_tag_value_,
    long embedding_tag_value_,
    long offset_tag_value_)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_scheduler(m_mesh)
    , input_tag_value(input_tag_value_)
    , embedding_tag_value(embedding_tag_value_)
    , offset_tag_value(offset_tag_value_)
{
    using namespace operations;
    // register the attributes
    m_position_handle = m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    m_vertex_tag_handle = m_mesh.get_attribute_handle<long>("m_vertex_tags", PrimitiveType::Vertex);
    m_edge_tag_handle = m_mesh.get_attribute_handle<long>("m_edge_tags", PrimitiveType::Edge);

    // offset corner case: resolution preprocess should be done in embedding part.
    // ...

    // offset Computation
    {
        OperationSettings<tri_mesh::EdgeSplitWithTag> split_edge_with_different_tag;
        split_edge_with_different_tag.position = m_position_handle;
        split_edge_with_different_tag.vertex_tag = m_vertex_tag_handle;
        split_edge_with_different_tag.edge_tag = m_edge_tag_handle;
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
        split_edge_with_same_tag.split_when_tags = TAGS_SAME;
        split_edge_with_same_tag.input_tag_value = input_tag_value;
        split_edge_with_same_tag.embedding_tag_value = embedding_tag_value;
        split_edge_with_same_tag.offset_tag_value = offset_tag_value;
        split_edge_with_same_tag.split_settings.split_boundary_edges = !m_lock_boundary;
        split_edge_with_same_tag.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeSplitWithTag>(
            "split_edge_with_same_tag_to_build_offset",
            split_edge_with_same_tag);
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
    }

    // // remeshing and optimization
    // {

    //     // swap
    //     // for exterior edges, it would be normal
    //     // for interior edges, offset vertices should never be swapped
    //     // and interior offset should be offset if its edge's length could be smaller
    //     OperationSettings<tri_mesh::EdgeSwap> swap_settings;
    //     swap_settings.must_improve_valence = true;
    //     m_scheduler.add_operation_type<tri_mesh::EdgeSwap>("swap", swap_settings);

    //     // relocate
    //     // exterior vertices just do averaging
    //     // offset average neighbours' position and push back to offset vertices
    //     // OperationSettings<tri_mesh::VertexSmooth> relocate_pass1;
    //     // relocate_pass1.for_extraction = true;
    //     // relocate_pass1.position = m_position_handle;
    //     // relocate_pass1.tag = m_tag_handle;
    //     // relocate_pass1.smooth_boundary = !m_lock_boundary;
    //     // m_scheduler.add_operation_type<tri_mesh::VertexSmooth>("relocate_pass1", relocate_pass1);

    //     OperationSettings<tri_mesh::PushOffset> relocate_pass2;
    //     relocate_pass2.distance = length;
    //     relocate_pass2.position = m_position_handle;
    //     relocate_pass2.tag = m_tag_handle;
    //     relocate_pass2.smooth_boundary = !m_lock_boundary;
    //     m_scheduler.add_operation_type<tri_mesh::PushOffset>("relocate_pass2", relocate_pass2);
    // }
}

void IsosurfaceExtraction::process(const long iteration_times)
{
    // build offset
    // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass1");
    // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass2");
    // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "buildoffset_pass3");
    // for (long i = 0; i < iteration_times; ++i) {
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "relocate_pass1");
    //     m_scheduler.run_operation_on_all(PrimitiveType::Edge, "relocate_pass2");
    // }
}

} // namespace wmtk::components::internal