#include <catch2/catch_test_macros.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

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
using namespace wmtk::simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;
TEST_CASE("consolidate_multimesh", "[mesh][consolidate_multimesh]")
{
    using namespace wmtk::operations;


    SECTION("1D")
    {
        DEBUG_EdgeMesh m = self_loop();
        REQUIRE(m.is_connectivity_valid());

        const int64_t edge_id = 0;
        Tuple edge = m.tuple_from_edge_id(edge_id);
        auto executor = m.get_emoe(edge);

        executor.split_edge();
        REQUIRE(m.is_connectivity_valid());
        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);

        multimesh::consolidate(m);
        REQUIRE(m.is_connectivity_valid());
    }


    SECTION("2D")
    {
        DEBUG_TriMesh m = hex_plus_two();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
        auto executor = m.get_tmoe(edge);
        EdgeCollapse collapse(m);
        collapse(simplex::Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());

        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(executor.flag_accessors[2].index_access().is_active(2) == false);
        REQUIRE(executor.flag_accessors[2].index_access().is_active(7) == false);
        CHECK(fv_accessor.vector_attribute(0)[1] == 5);
        CHECK(fv_accessor.vector_attribute(1)[0] == 5);
        CHECK(fv_accessor.vector_attribute(3)[0] == 5);
        CHECK(fv_accessor.vector_attribute(5)[2] == 5);
        CHECK(fv_accessor.vector_attribute(6)[2] == 5);
        CHECK(fv_accessor.vector_attribute(4)[0] == 5);

        REQUIRE(m.is_connectivity_valid());
        multimesh::consolidate(m);
        REQUIRE(m.is_connectivity_valid());
    }

    SECTION("3D")
    {
        wmtk::tests_3d::DEBUG_TetMesh m = wmtk::tests_3d::one_ear();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.face_tuple_with_vs_and_t(1, 2, 0, 0);
        EdgeCollapse collapse(m);
        collapse(simplex::Simplex::edge(m, edge));
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);

        multimesh::consolidate(m);
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.valid_primitive_count(PT) == 1);
    }
    SECTION("MM")
    {
        DEBUG_TriMesh parent = two_neighbors();
        std::shared_ptr<DEBUG_TriMesh> child0_ptr =
            std::make_shared<DEBUG_TriMesh>(single_triangle());
        std::shared_ptr<DEBUG_TriMesh> child1_ptr = std::make_shared<DEBUG_TriMesh>(one_ear());
        std::shared_ptr<DEBUG_TriMesh> child2_ptr =
            std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

        auto& child0 = *child0_ptr;
        auto& child1 = *child1_ptr;
        auto& child2 = *child2_ptr;

        parent.reserve_more_attributes({10, 10, 10});
        child0.reserve_more_attributes({10, 10, 10});
        child1.reserve_more_attributes({10, 10, 10});
        child2.reserve_more_attributes({10, 10, 10});

        auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0});
        auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0, 1});
        auto child2_map = multimesh::same_simplex_dimension_surjection(parent, child2, {0, 1, 2});


        parent.register_child_mesh(child0_ptr, child0_map);
        parent.register_child_mesh(child1_ptr, child1_map);
        parent.register_child_mesh(child2_ptr, child2_map);

        Tuple edge = parent.edge_tuple_with_vs_and_t(1, 0, 1);
        operations::EdgeSplit split(parent);
        REQUIRE(!split(Simplex::edge(parent, edge)).empty());

        multimesh::consolidate(parent);
        REQUIRE(parent.is_connectivity_valid());
        REQUIRE(child0.is_connectivity_valid());
        REQUIRE(child1.is_connectivity_valid());
        REQUIRE(child2.is_connectivity_valid());
    }
}


TEST_CASE("consolidate_multimesh_splits", "[mesh][consolidate_multimesh]")
{
    using namespace wmtk::operations;
    int number = 5;
    auto dptr = disk_to_individual_multimesh(number);

    auto& c = dptr->get_multi_mesh_child_mesh({0});

    operations::EdgeSplit split_op(*dptr);


    for (int j = 0; j < 3; ++j) {
        for (const auto& tup : dptr->get_all(wmtk::PrimitiveType::Edge)) {
            if (dptr->is_valid(tup)) {
                split_op(simplex::Simplex::edge(*dptr, tup)); //.empty();
            }
        }
    }
    DEBUG_TriMesh& debug_d = reinterpret_cast<DEBUG_TriMesh&>(*dptr);
    DEBUG_TriMesh& debug_i = reinterpret_cast<DEBUG_TriMesh&>(c);

    multimesh::consolidate(*dptr);
    debug_d.multi_mesh_manager().check_map_valid(debug_d);
    debug_i.multi_mesh_manager().check_map_valid(debug_i);
    for (int j = 0; j < 3; ++j) {
        for (const auto& tup : dptr->get_all(wmtk::PrimitiveType::Edge)) {
            if (dptr->is_valid(tup)) {
                split_op(simplex::Simplex::edge(*dptr, tup));
            }
        }
    }
}
