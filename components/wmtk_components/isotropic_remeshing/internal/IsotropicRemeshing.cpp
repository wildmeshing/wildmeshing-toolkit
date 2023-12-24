#include "IsotropicRemeshing.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

IsotropicRemeshing::IsotropicRemeshing(
    TriMesh& mesh,
    const double length,
    const bool lock_boundary,
    const bool preserve_childmesh_topology,
    const bool preserve_childmesh_geometry,
    const bool do_split,
    const bool do_collapse,
    const bool do_swap,
    const bool do_smooth,
    const bool debug_output)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
    , m_preserve_childmesh_topology{preserve_childmesh_topology}
    , m_preserve_childmesh_geometry{preserve_childmesh_geometry}
    , m_do_split{do_split}
    , m_do_collapse{do_collapse}
    , m_do_swap{do_swap}
    , m_do_smooth{do_smooth}
    , m_debug_output{debug_output}
    , m_position_handle{m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex)}
{
    using namespace operations;
    // split
    {
        // OperationSettings<tri_mesh::EdgeSplitAtMidpoint> split_settings(m_mesh);
        // split_settings.position = m_position_handle;
        //// thresholds are inverted because we continue splitting because we
        //// always split until we're under this length, which is the max
        //// required length for the op to happen
        // split_settings.min_squared_length = m_length_max * m_length_max;
        // split_settings.split_boundary_edges = !m_lock_boundary;
        //
        // m_scheduler.add_operation_type<tri_mesh::EdgeSplitAtMidpoint>("split", split_settings);
    } // collapse
    {
        // OperationSettings<tri_mesh::EdgeCollapseToMidpoint> op_settings(m_mesh);
        // op_settings.position = m_position_handle;
        //// thresholds are inverted because we continue collapsing because we
        //// always collapse until we're over this length, which is the minimum
        //// required length for the op to happen
        // op_settings.max_squared_length = m_length_min * m_length_min;
        // op_settings.collapse_boundary_edges = !m_lock_boundary;
        // op_settings.preserve_topology = m_preserve_childmesh_topology;
        // op_settings.preserve_geometry = m_preserve_childmesh_geometry;
        // op_settings.collapse_towards_boundary = true;
        //
        // m_scheduler.add_operation_type<tri_mesh::EdgeCollapseToMidpoint>("collapse",
        // op_settings);
    } // flip
    {
        // OperationSettings<tri_mesh::EdgeSwapValence> op_settings(m_mesh);
        //
        // m_scheduler.add_operation_type<tri_mesh::EdgeSwapValence>("swap", op_settings);
    } // smooth
    {
        // OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth> op_settings(m_mesh);
        // op_settings.position = m_position_handle;
        // op_settings.smooth_boundary = !m_lock_boundary;
        //
        // m_scheduler.add_operation_type<tri_mesh::VertexTangentialLaplacianSmooth>(
        //     "smooth",
        //     op_settings);
    }
}

void IsotropicRemeshing::remeshing(const long iterations)
{
    // debug write
    // ParaviewWriter writer("remeshing_test_circle_0", "vertices", m_mesh, true, true, true,
    // false); m_mesh.serialize(writer);

    auto child_meshes = m_mesh.get_child_meshes();

    auto child_vertex_handle =
        child_meshes[0]->register_attribute<double>("vertices", wmtk::PrimitiveType::Vertex, 3);
    auto child_vertex_accessor = child_meshes[0]->create_accessor(child_vertex_handle);

    auto parent_vertex_handle =
        m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto parent_vertex_accessor = m_mesh.create_accessor(parent_vertex_handle);

    for (long i = 0; i < iterations; ++i) {
        bool is_conn_valid;
        bool is_map_valid;


        wmtk::logger().info("Iteration {}", i);

        if (m_do_split) {
            // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "split");
            is_conn_valid = m_mesh.is_connectivity_valid();
            wmtk::logger().info("Is connectivity valid: {}", is_conn_valid);
            if (!is_conn_valid) throw std::runtime_error("invalid mesh connectivty");
            wmtk::logger().info("Done split {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            wmtk::io::ParaviewWriter writer1(
                "remeshing_test_circle_" + std::to_string(i * 4 + 1),
                "vertices",
                m_mesh,
                true,
                true,
                true,
                false);

            m_mesh.serialize(writer1);

            for (const auto v : child_meshes[0]->get_all(PrimitiveType::Vertex)) {
                auto parent_v =
                    child_meshes[0]->map_to_root_tuple(Simplex(PrimitiveType::Vertex, v));
                child_vertex_accessor.vector_attribute(v) =
                    parent_vertex_accessor.vector_attribute(parent_v);
            }

            ParaviewWriter writer_c1(
                "remeshing_test_circle_child_" + std::to_string(i * 4 + 1),
                "vertices",
                *(child_meshes[0]),
                true,
                true,
                false,
                false);

            child_meshes[0]->serialize(writer_c1);
        }

        if (m_do_collapse) {
            // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "collapse");
            is_conn_valid = m_mesh.is_connectivity_valid();
            wmtk::logger().info("Is connectivity valid: {}", is_conn_valid);
            if (!is_conn_valid) throw std::runtime_error("invalid mesh connectivty");

            wmtk::logger().info("Done collapse {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            ParaviewWriter writer2(
                "remeshing_test_circle_" + std::to_string(i * 4 + 2),
                "vertices",
                m_mesh,
                true,
                true,
                true,
                false);

            m_mesh.serialize(writer2);

            for (const auto v : child_meshes[0]->get_all(PrimitiveType::Vertex)) {
                auto parent_v =
                    child_meshes[0]->map_to_root_tuple(Simplex(PrimitiveType::Vertex, v));
                child_vertex_accessor.vector_attribute(v) =
                    parent_vertex_accessor.vector_attribute(parent_v);

                long parent_vid = m_mesh._debug_id(parent_v, PrimitiveType::Vertex);
                EdgeMesh& child_em = dynamic_cast<EdgeMesh&>(*(child_meshes[0]));
                long child_vid = child_em._debug_id(v, PrimitiveType::Vertex);
                continue;
            }

            ParaviewWriter writer_c2(
                "remeshing_test_circle_child_" + std::to_string(i * 4 + 2),
                "vertices",
                *(child_meshes[0]),
                true,
                true,
                false,
                false);

            child_meshes[0]->serialize(writer_c2);
        }

        if (m_do_swap) {
            // m_scheduler.run_operation_on_all(PrimitiveType::Edge, "swap");
            is_conn_valid = m_mesh.is_connectivity_valid();
            if (!is_conn_valid) throw std::runtime_error("invalid mesh connectivty");

            wmtk::logger().info("Is connectivity valid: {}", is_conn_valid);
            wmtk::logger().info("Done swap {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            ParaviewWriter writer3(
                "remeshing_test_circle_" + std::to_string(i * 4 + 3),
                "vertices",
                m_mesh,
                true,
                true,
                true,
                false);

            m_mesh.serialize(writer3);

            for (const auto v : child_meshes[0]->get_all(PrimitiveType::Vertex)) {
                auto parent_v =
                    child_meshes[0]->map_to_root_tuple(Simplex(PrimitiveType::Vertex, v));
                child_vertex_accessor.vector_attribute(v) =
                    parent_vertex_accessor.vector_attribute(parent_v);
            }

            ParaviewWriter writer_c3(
                "remeshing_test_circle_child_" + std::to_string(i * 4 + 3),
                "vertices",
                *(child_meshes[0]),
                true,
                true,
                false,
                false);

            child_meshes[0]->serialize(writer_c3);
        }

        if (m_do_smooth) {
            // m_scheduler.run_operation_on_all(PrimitiveType::Vertex, "smooth");
            // is_conn_valid = m_mesh.is_connectivity_valid();
            // if (!is_conn_valid) throw std::runtime_error("invalid mesh connectivty");

            // wmtk::logger().info("Is connectivity valid: {}", is_conn_valid);
            // wmtk::logger().info("Done smooth {}\n", i);
        }

        // debug write
        if (m_debug_output) {
            ParaviewWriter writer4(
                "remeshing_test_circle_" + std::to_string(i * 4 + 4),
                "vertices",
                m_mesh,
                true,
                true,
                true,
                false);

            m_mesh.serialize(writer4);

            for (const auto v : child_meshes[0]->get_all(PrimitiveType::Vertex)) {
                auto parent_v =
                    child_meshes[0]->map_to_root_tuple(Simplex(PrimitiveType::Vertex, v));
                child_vertex_accessor.vector_attribute(v) =
                    parent_vertex_accessor.vector_attribute(parent_v);
            }

            ParaviewWriter writer_c4(
                "remeshing_test_circle_child_" + std::to_string(i * 4 + 4),
                "vertices",
                *(child_meshes[0]),
                true,
                true,
                false,
                false);

            child_meshes[0]->serialize(writer_c4);
        }
    }
}

} // namespace wmtk::components::internal
