

#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include <array>

using namespace wmtk;
using namespace wmtk::tests;


TEST_CASE("test_execution_single_triangle") {

    DEBUG_TriMesh m;
    m = single_triangle();
    Executor executor(m);
    executor.add_operation<TriMeshSplitOperation>("edge_split");


    executor.run_all(PrimitiveType::Edge, "edge_split");

    TriMesh m2 = single_triangle();
    CHECK(m != m2);
}
