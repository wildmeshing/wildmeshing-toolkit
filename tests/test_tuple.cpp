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

TEST_CASE("switch_vertex", "[test_tuple]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_edge(0, 0);
    REQUIRE(tuple.vid(mesh) == 0);

    const auto t1 = mesh.switch_vertex(tuple);
    REQUIRE(t1.vid(mesh) == 1);
    int eid1 = tuple.eid(mesh);
    int eid2 = t1.eid(mesh);
    REQUIRE(eid1 == eid2);

    const auto t2 = mesh.switch_vertex(t1);
    REQUIRE(tuple.vid(mesh) == t2.vid(mesh));
}

TEST_CASE("switch_edge", "[test_tuple]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_vertex(0);

    int eid1 = tuple.eid(mesh);
    const auto t1_tmp = mesh.switch_edge(tuple);
    const auto t1 = mesh.switch_vertex(t1_tmp);
    const auto t2_tmp = mesh.switch_edge(t1);
    const auto t2 = mesh.switch_vertex(t2_tmp);
    const auto t3 = mesh.switch_edge(t2);
    int eid2 = t3.eid(mesh);
    REQUIRE(eid1 == eid2);
}

TEST_CASE("switch_face", "[test_tuple]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_vertex(0);

    int fid1 = tuple.fid(mesh);
    const auto t1 = mesh.switch_face(tuple);
    const auto t2 = mesh.switch_face(t1);
    const auto t3 = mesh.switch_face(t2);
    const auto t4 = mesh.switch_face(t3);
    int fid2 = t4.fid(mesh);
    REQUIRE(fid1 == fid2);
}

TEST_CASE("switch_tet", "[test_tuple]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);

    int tid1 = tuple.tid(mesh);
    const auto t1 = mesh.switch_tetrahedron(tuple);
    REQUIRE(t1.has_value());
    const auto t2 = mesh.switch_tetrahedron(t1.value());
    REQUIRE(t2.has_value());
    int tid2 = t2.value().tid(mesh);
    REQUIRE(tid1 == tid2);
}
