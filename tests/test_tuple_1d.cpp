#include <stdlib.h>
#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("1D_initialize", "[mesh_creation],[tuple_1d]")
{
    DEBUG_EdgeMesh m;
    RowVectors2l edges;
    edges.resize(1, 2);
    edges << 0, 1;

    SECTION("init with FV, FE, FF, VF, EF")
    {
        m = single_triangle();
    }
    SECTION("init directly from RowVectors3l")
    {
        m.initialize(tris);
    }

    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 3);
    const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
    REQUIRE(edges.size() == 3);
    const std::vector<Tuple> faces = m.get_all(PrimitiveType::Face);
    REQUIRE(faces.size() == 1);


    REQUIRE(m.is_connectivity_valid());

    const Tuple t = m.get_all(PrimitiveType::Face)[0];
    REQUIRE(m.is_valid_slow(t));
}