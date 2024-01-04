#include <array>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/VertexLaplacianSmooth.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::tests;


TEST_CASE("test_execution_single_triangle", "[scheduler][2D]")
{
    DEBUG_TriMesh m;
    m = single_triangle();

    Scheduler scheduler;
    operations::EdgeSplit op(m);
    scheduler.run_operation_on_all(op);


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
        auto pos = m.create_accessor<double>(pos_handle);
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            pos.vector_attribute(v) = Eigen::Vector3d{0, 0, 0};
        }
    }

    Scheduler scheduler;
    operations::VertexLaplacianSmooth op(
        m,
        m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex).as<double>());
    op.add_invariant(std::make_shared<InteriorVertexInvariant>(m));
    scheduler.run_operation_on_all(op);
}

TEST_CASE("scheduler_success_report", "[scheduler][operations][2D][.]")
{
    // TODOfix test on something else than smoothing

    using namespace operations;

    SECTION("single_run")
    {
        DEBUG_TriMesh m;
        int64_t expected_op_success = -1;
        int64_t expected_op_fail = -1;
        std::unique_ptr<operations::VertexLaplacianSmooth> op;
        SECTION("single_triangle_with_boundary")
        {
            m = single_equilateral_triangle();
            expected_op_success = 1;
            expected_op_fail = 2;
            op = std::make_unique<operations::VertexLaplacianSmooth>(
                m,
                m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex).as<double>());
        }
        SECTION("single_triangle_without_boundary")
        {
            m = single_equilateral_triangle();
            expected_op_success = 0;
            expected_op_fail = 3;
            op = std::make_unique<operations::VertexLaplacianSmooth>(
                m,
                m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex).as<double>());
            op->add_invariant(std::make_shared<InteriorVertexInvariant>(m));
        }
        // SECTION("edge_region_with_boundary")
        // {
        //     m = edge_region_with_position();
        //     expected_op_success = 2;
        //     expected_op_fail = 8;
        //     op = std::make_unique<operations::VertexLaplacianSmooth>(
        //         m,
        //         m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));
        // }

        const int64_t expected_op_sum = expected_op_success + expected_op_fail;

        Scheduler scheduler;
        auto res = scheduler.run_operation_on_all(*op);

        CHECK(res.number_of_performed_operations() == expected_op_sum);
        CHECK(res.number_of_successful_operations() == expected_op_success);
        CHECK(res.number_of_failed_operations() == expected_op_fail);
    }

    SECTION("multiple_runs")
    {
        DEBUG_TriMesh m = single_equilateral_triangle();
        operations::VertexLaplacianSmooth op(
            m,
            m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex).as<double>());

        Scheduler scheduler;

        for (size_t i = 0; i < 3; ++i) {
            auto res = scheduler.run_operation_on_all(op);
            CHECK(res.number_of_performed_operations() == 3);
            CHECK(res.number_of_successful_operations() == 1);
            CHECK(res.number_of_failed_operations() == 2);
        }
    }
}
