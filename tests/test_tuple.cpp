#include <wmtk/TetMesh.h>

#include <catch2/catch.hpp>
#include <iostream>

using namespace wmtk;

TEST_CASE("test_get_edges", "[test_tuple]")
{
	TetMesh mesh;
	mesh.init(4, {{{0, 1, 2, 3}}});
	const auto edges = mesh.get_edges();

	REQUIRE(edges.size() == 6);
}

TEST_CASE("swith_vertex", "[test_tuple]")
{
	TetMesh mesh;
	mesh.init(4, {{{0, 1, 2, 3}}});
	const auto tuple = mesh.tuple_from_edge(0, 0);
	REQUIRE(tuple.vid() == 0);

	const auto t1 = mesh.switch_vertex(tuple);
	REQUIRE(t1.vid() == 1);

	const auto t2 = mesh.switch_vertex(t1);
	REQUIRE(tuple.vid() == t2.vid());
}


