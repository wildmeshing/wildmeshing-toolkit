#include <catch2/catch_test_macros.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>

#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/redirect_logger_to_cout.hpp"

#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace operations;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("test_mesh_virtuals", "[mesh]")
{
    wmtk::PointMesh pm;
    wmtk::EdgeMesh em;
    wmtk::TriMesh fm;
    wmtk::TetMesh tm;

    REQUIRE(pm.top_cell_dimension() == 0);
    REQUIRE(em.top_cell_dimension() == 1);
    REQUIRE(fm.top_cell_dimension() == 2);
    REQUIRE(tm.top_cell_dimension() == 3);

    REQUIRE(pm.top_simplex_type() == wmtk::PrimitiveType::Vertex);
    REQUIRE(em.top_simplex_type() == wmtk::PrimitiveType::Edge);
    REQUIRE(fm.top_simplex_type() == wmtk::PrimitiveType::Face);
    REQUIRE(tm.top_simplex_type() == wmtk::PrimitiveType::Tetrahedron);
}

TEST_CASE("consolidate", "[mesh][consolidate]")
{
    using namespace wmtk::operations;


    SECTION("1D")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const long edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        REQUIRE(m.is_valid(edge, hash_accessor));

        auto executor = m.get_emoe(edge, hash_accessor);

        executor.split_edge();
        REQUIRE(m.is_connectivity_valid());
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);

        m.consolidate();
        REQUIRE(m.is_connectivity_valid());
    }


    SECTION("2D")
    {
        DEBUG_TriMesh m = hex_plus_two();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(edge));
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(2)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(7)) == 0);
        CHECK(fv_accessor.vector_attribute(0)[1] == 5);
        CHECK(fv_accessor.vector_attribute(1)[0] == 5);
        CHECK(fv_accessor.vector_attribute(3)[0] == 5);
        CHECK(fv_accessor.vector_attribute(5)[2] == 5);
        CHECK(fv_accessor.vector_attribute(6)[2] == 5);
        CHECK(fv_accessor.vector_attribute(4)[0] == 5);

        REQUIRE(m.is_connectivity_valid());
        m.consolidate();
        REQUIRE(m.is_connectivity_valid());
    }

    SECTION("3D")
    {
        wmtk::tests_3d::DEBUG_TetMesh m = wmtk::tests_3d::one_ear();
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0, 0);
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);

        m.consolidate();
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);
    }
}
