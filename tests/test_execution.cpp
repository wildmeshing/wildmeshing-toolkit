#include <spdlog/spdlog.h>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexSmooth.hpp>
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
    scheduler.add_operation_type<operations::tri_mesh::EdgeSplit>("edge_split");


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
        auto pos_handle = m.register_attribute<double>("position", PrimitiveType::Vertex, 3);
        auto pos = m.create_accessor(pos_handle);
        for (const Tuple& v : m.get_all(PrimitiveType::Vertex)) {
            pos.vector_attribute(v) = Eigen::Vector3d{0, 0, 0};
        }
    }

    OperationSettings<tri_mesh::VertexSmooth> op_settings;
    op_settings.position = m.get_attribute_handle<double>("position", PrimitiveType::Vertex);

    Scheduler scheduler(m);
    scheduler.add_operation_type<tri_mesh::VertexSmooth>("vertex_smooth", op_settings);

    scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
}