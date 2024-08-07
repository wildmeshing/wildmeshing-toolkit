
#include <catch2/catch_test_macros.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>

#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
#include "tools/is_free.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace operations;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("test_debug_free", "[mesh]")
{
    CHECK(is_free(single_line()));
    CHECK_FALSE(is_free(two_segments()));
    CHECK_FALSE(is_free(multiple_lines()));
    CHECK_FALSE(is_free(two_line_loop()));
    //

    CHECK(is_free(single_triangle()));
    CHECK_FALSE(is_free(one_ear()));
    CHECK_FALSE(is_free(quad()));
    CHECK_FALSE(is_free(two_neighbors()));
    CHECK_FALSE(is_free(two_neighbors_plus_one()));
    CHECK_FALSE(is_free(edge_region()));
    CHECK_FALSE(is_free(*disk(3)));
    CHECK(is_free(*individual_triangles(3)));
}
