#include <spdlog/spdlog.h>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmooth.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::tests;


TEST_CASE("test_execution_single_triangle", "[scheduler][2D]")
{
    DEBUG_TriMesh m;
    m = single_triangle();
    Scheduler scheduler(m);
    operations::OperationSettings<operations::tri_mesh::EdgeSplit> op_settings(m);

    scheduler.add_operation_type<operations::tri_mesh::EdgeSplit>("edge_split", op_settings);

    scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");


    DEBUG_TriMesh m2 = single_triangle();
    CHECK(m != m2);
}

TEST_CASE("operation_with_settings", "[scheduler][operations][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m;
    SECTION("single_triangle")
    {
        m = single_triangle();
    }
    SECTION("edge_region")
    {
        m = edge_region();
    }
    {
        // assign positions
        auto pos_handle = m.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
        auto pos = m.create_accessor(pos_handle);
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            pos.vector_attribute(v) = Eigen::Vector3d{0, 0, 0};
        }
    }

    operations::OperationSettings<tri_mesh::VertexLaplacianSmooth> op_settings(m);
    op_settings.position = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    Scheduler scheduler(m);
    scheduler.add_operation_type<tri_mesh::VertexLaplacianSmooth>("vertex_smooth", op_settings);

    scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
}

TEST_CASE("scheduler_success_report", "[scheduler][operations][2D]")
{
    using namespace operations;

    SECTION("single_run")
    {
        DEBUG_TriMesh m;
        long expected_op_success = -1;
        long expected_op_fail = -1;
        operations::OperationSettings<tri_mesh::VertexLaplacianSmooth> op_settings(m);
        SECTION("single_triangle_with_boundary")
        {
            m = single_equilateral_triangle();
            expected_op_success = 1;
            expected_op_fail = 2;
            op_settings.smooth_boundary = true;
        }
        SECTION("single_triangle_without_boundary")
        {
            m = single_equilateral_triangle();
            expected_op_success = 0;
            expected_op_fail = 3;
            op_settings.smooth_boundary = false;
        }
        // SECTION("edge_region_with_boundary")
        //{
        //     m = edge_region_with_position();
        //     expected_op_success = 2;
        //     expected_op_fail = 8;
        //     op_settings.smooth_boundary = true;
        // }
        const long expected_op_sum = expected_op_success + expected_op_fail;

        op_settings.position = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        Scheduler scheduler(m);
        scheduler.add_operation_type<tri_mesh::VertexLaplacianSmooth>("vertex_smooth", op_settings);

        scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");

        CHECK(scheduler.number_of_performed_operations() == expected_op_sum);
        // CHECK(scheduler.number_of_successful_operations() == expected_op_success);
        // CHECK(scheduler.number_of_failed_operations() == expected_op_fail);
    }
    // SECTION("multiple_runs")
    // {
    //     DEBUG_TriMesh m = single_equilateral_triangle();
    //     operations::OperationSettings<tri_mesh::VertexLaplacianSmooth> op_settings(m);
    //     op_settings.smooth_boundary = true;
    //     op_settings.position = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    //     Scheduler scheduler(m);
    //     scheduler.add_operation_type<tri_mesh::VertexLaplacianSmooth>("vertex_smooth",
    //     op_settings);

    //     for (size_t i = 0; i < 3; ++i) {
    //         scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
    //         CHECK(scheduler.number_of_performed_operations() == 3);
    //         CHECK(scheduler.number_of_successful_operations() == 1);
    //         CHECK(scheduler.number_of_failed_operations() == 2);
    //     }
    // }
}
