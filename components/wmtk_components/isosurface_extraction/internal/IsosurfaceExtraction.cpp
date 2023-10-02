#include "IsosurfaceExtraction.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialSmooth.hpp>

namespace wmtk::components::internal {

IsosurfaceExtraction::IsosurfaceExtraction(
    TriMesh& mesh,
    const double length,
    const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_position_handle{m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex)}
    , m_scheduler(m_mesh)
{
    using namespace operations;
    // split
    {
        OperationSettings<tri_mesh::EdgeSplitAtMidpoint> split_settings;
        split_settings.position = m_position_handle;
        // thresholds are inverted because we continue splitting because we
        // always split until we're under this length, which is the max
        // required length for the op to happen
        split_settings.min_squared_length = m_length_max * m_length_max;
        split_settings.split_settings.split_boundary_edges = !m_lock_boundary;
        split_settings.initialize_invariants(m_mesh);

        m_scheduler.add_operation_type<tri_mesh::EdgeSplitAtMidpoint>("split", split_settings);
    }
    // collapse
    {
        OperationSettings<tri_mesh::EdgeCollapseToMidpoint> op_settings;
        op_settings.position = m_position_handle;
        // thresholds are inverted because we continue collapsing because we
        // always collapse until we're over this length, which is the minimum
        // required length for the op to happen
        op_settings.max_squared_length = m_length_min * m_length_min;
        op_settings.collapse_settings.collapse_boundary_edges = !m_lock_boundary;
        op_settings.collapse_towards_boundary = true;

        op_settings.initialize_invariants(m_mesh);
        m_scheduler.add_operation_type<tri_mesh::EdgeCollapseToMidpoint>("collapse", op_settings);
    }
    // flip
    {
        OperationSettings<tri_mesh::EdgeSwap> op_settings;
        op_settings.must_improve_valence = true;

        m_scheduler.add_operation_type<tri_mesh::EdgeSwap>("swap", op_settings);
    }
    // smooth
    {
        OperationSettings<tri_mesh::VertexTangentialSmooth> op_settings;
        op_settings.smooth_settings.position = m_position_handle;
        op_settings.smooth_settings.smooth_boundary = !m_lock_boundary;

        m_scheduler.add_operation_type<tri_mesh::VertexTangentialSmooth>("smooth", op_settings);
    }
}

void IsosurfaceExtraction::remeshing(const long iterations)
{
    for (long i = 0; i < iterations; ++i) {
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
        m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
        m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "smooth");
    }
}

} // namespace wmtk::components::internal
