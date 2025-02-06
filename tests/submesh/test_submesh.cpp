#include <catch2/catch_test_macros.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

#include "tools/DEBUG_EdgeMesh.hpp"

using namespace wmtk;
using namespace submesh;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("submesh_init", "[mesh][submesh]")
{
    // logger().set_level(spdlog::level::off);

    // basic test for implementing
    std::shared_ptr<tests::DEBUG_TriMesh> mesh_in =
        std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position());

    tests::DEBUG_TriMesh& m = *mesh_in;
    const Tuple edge45 = m.edge_tuple_from_vids(4, 5);

    Embedding emb(mesh_in);
    std::shared_ptr<SubMesh> sub_ptr = emb.add_submesh();
    SubMesh& sub = *sub_ptr;

    CHECK_THROWS(sub.top_simplex_type());

    sub.add_simplex(edge45, PE);

    CHECK(sub.contains(edge45, PE));
    CHECK(sub.contains(edge45, PV));
    CHECK(sub.contains(sub.switch_tuple(edge45, PV), PV));
    CHECK(sub.is_boundary(edge45, PE));
    CHECK(sub.is_boundary(edge45, PV));
    CHECK(sub.is_boundary(sub.switch_tuple(edge45, PV), PV));
    CHECK(sub.top_simplex_type(edge45) == PE);

    // Test switch_tuple_vector
    const Tuple edge34 = m.edge_tuple_from_vids(3, 4);
    sub.add_simplex(edge34, PE);
    CHECK(sub.contains(edge34, PE));
    CHECK(sub.contains(edge34, PV));
    CHECK(sub.contains(sub.switch_tuple(edge34, PV), PV));
    CHECK(sub.is_boundary(edge34, PV));
    CHECK(!sub.is_boundary(sub.switch_tuple(edge34, PV), PV));
    CHECK(sub.top_simplex_type(edge34) == PE);

    // switch from edge 45 to 43
    {
        CHECK_THROWS(sub.switch_tuple(edge34, PE));
        CHECK_THROWS(sub.switch_tuple_vector(edge45, PV).size() == 1);
        std::vector<Tuple> v4_edges;
        REQUIRE_NOTHROW(v4_edges = sub.switch_tuple_vector(edge45, PE));
        CHECK(v4_edges.size() == 2);
        CHECK(v4_edges[0] == edge45);
        CHECK(m.get_id_simplex(v4_edges[1], PE) == m.get_id_simplex(edge34, PE));
        CHECK(m.get_id_simplex(v4_edges[1], PV) == m.get_id_simplex(edge45, PV));
    }

    const Tuple edge04 = m.edge_tuple_from_vids(0, 4);
    sub.add_simplex(edge04, PE);
    CHECK(sub.is_boundary(edge04, PV));
    CHECK(!sub.is_boundary(sub.switch_tuple(edge04, PV), PV));
    // switch from edge 45 to all others
    {
        std::vector<Tuple> v4_edges;
        REQUIRE_NOTHROW(v4_edges = sub.switch_tuple_vector(edge45, PE));
        CHECK(v4_edges.size() == 3);
        CHECK(v4_edges[0] == edge45);
        for (const Tuple& t : v4_edges) {
            CHECK(m.get_id_simplex(t, PV) == m.get_id_simplex(edge45, PV));
        }
    }

    const Tuple edge54 = sub.switch_tuple(edge45, PV);
    {
        std::vector<Tuple> v5_edges;
        REQUIRE_NOTHROW(v5_edges = sub.switch_tuple_vector(edge54, PE));
        CHECK(v5_edges.size() == 1);
        CHECK(v5_edges[0] == edge54);
    }

    const Tuple face596 = m.face_tuple_from_vids(5, 9, 6);
    sub.add_simplex(face596, PF);
    CHECK(sub.top_simplex_type() == PF);
    CHECK(sub.id(face596, PF) == 9);
    CHECK(sub.is_boundary(face596, PF));
    CHECK(sub.is_boundary(m.vertex_tuple_from_id(5), PV));
    const Tuple edge59 = m.edge_tuple_with_vs_and_t(5, 9, 9);
    {
        std::vector<Tuple> v5_edges;
        REQUIRE_NOTHROW(v5_edges = sub.switch_tuple_vector(edge59, PE));
        CHECK(v5_edges.size() == 3);
        CHECK(v5_edges[0] == edge59);
        for (const Tuple& t : v5_edges) {
            CHECK(m.get_id_simplex(t, PV) == m.get_id_simplex(edge59, PV));
        }

        // local switch
        CHECK_NOTHROW(sub.switch_tuple(edge59, PE));

        // face switch
        CHECK(sub.switch_tuple_vector(face596, PF).size() == 1);
    }


    {
        ParaviewWriter writer("submesh_init", "vertices", m, false, true, true, false);
        CHECK_NOTHROW(m.serialize(writer));
    }
}

TEST_CASE("submesh_init_from_tag", "[mesh][submesh]")
{
    // logger().set_level(spdlog::level::off);

    std::shared_ptr<tests::DEBUG_TriMesh> mesh_in =
        std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position());

    tests::DEBUG_TriMesh& m = *mesh_in;

    Embedding emb(mesh_in);
    std::shared_ptr<SubMesh> sub_ptr = emb.add_submesh();
    SubMesh& sub = *sub_ptr;

    // register tag attribute
    {
        auto tag_handle = m.register_attribute<int64_t>("tag", PF, 1);
        auto acc = m.create_accessor<int64_t>(tag_handle);
        acc.scalar_attribute(m.tuple_from_face_id(0)) = 1;
        acc.scalar_attribute(m.tuple_from_face_id(2)) = 1;

        sub.add_from_tag_attribute<int64_t>(tag_handle.as<int64_t>(), 1);
    }

    CHECK(sub.top_simplex_type() == PF);
    CHECK(sub.get_all(PF).size() == 2);
    CHECK(sub.get_all(PE).size() == 6);
    CHECK(sub.get_all(PV).size() == 5);
}

TEST_CASE("submesh_top_dimension_cofaces", "[mesh][submesh]")
{
    REQUIRE(false); // test not implemented

    // logger().set_level(spdlog::level::off);

    // basic test for implementing
    std::shared_ptr<tests::DEBUG_TriMesh> mesh_in =
        std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position());

    tests::DEBUG_TriMesh& m = *mesh_in;
    const Tuple edge45 = m.edge_tuple_from_vids(4, 5);

    Embedding emb(mesh_in);
    std::shared_ptr<SubMesh> sub_ptr = emb.add_submesh();
    SubMesh& sub = *sub_ptr;

    CHECK_THROWS(sub.top_simplex_type());

    sub.add_simplex(edge45, PE);
}