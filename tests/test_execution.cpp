

#include <spdlog/spdlog.h>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/operations/TriMeshCollapseEdgeOperation.hpp>
#include <wmtk/operations/TriMeshSplitEdgeOperation.hpp>
#include <wmtk/operations/TriMeshVertexSmoothOperation.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;


TEST_CASE("test_execution_single_triangle", "[scheduler],[2D]")
{
    DEBUG_TriMesh m;
    m = single_triangle();
    Scheduler scheduler(m);
    scheduler.add_operation_type<TriMeshSplitEdgeOperation>("edge_split");


    scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");


    DEBUG_TriMesh m2 = single_triangle();
    CHECK(m != m2);
}

TEST_CASE("factory_with_handles", "[scheduler],[2D]")
{
    DEBUG_TriMesh m;
    m = single_triangle();

    TriMeshVertexSmoothOperation::Handles handles{
        m.get_attribute_handle<double>("position", PrimitiveType::Vertex)};

    Scheduler scheduler(m);
    scheduler
        .add_operation_type<TriMeshVertexSmoothOperation, TriMeshVertexSmoothOperation::Handles>(
            "vertex_smooth",
            handles);


    scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");


    DEBUG_TriMesh m2 = single_triangle();
    CHECK(m != m2);
}