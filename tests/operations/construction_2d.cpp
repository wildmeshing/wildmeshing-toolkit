#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"


using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;

namespace {

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
// constexpr PrimitiveType PF = PrimitiveType::Face;

// because TriMesh::split_edge isn'ta waare of preconditions we need to tell the system whether
// something should succeed
void test_split(DEBUG_TriMesh& m, const Tuple& e, bool should_succeed)
{
    using namespace operations;


    EdgeSplit op(m);
    bool result = !op(Simplex::edge(m, e)).empty(); // should run the split
    REQUIRE(should_succeed == result);
}
void test_split(DEBUG_TriMesh& mesh, int64_t edge_index, bool should_succeed)
{
    Tuple e = mesh.tuple_from_id(PE, edge_index);
    REQUIRE(mesh.id(e, PE) == edge_index);
    test_split(mesh, e, should_succeed);
}

// because TriMesh::collapse_edge isn'ta waare of preconditions we need to tell the system whether
// something should succeed
void test_collapse(DEBUG_TriMesh& m, const Tuple& e, bool should_succeed)
{
    using namespace operations;

    EdgeCollapse op(m);
    op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));


    bool result = !op(Simplex::edge(m, e)).empty(); // should run the split
    REQUIRE(m.is_connectivity_valid());
    REQUIRE(should_succeed == result);

}
void test_collapse(DEBUG_TriMesh& mesh, int64_t edge_index, bool should_succeed)
{
    Tuple e = mesh.tuple_from_id(PE, edge_index);
    REQUIRE(mesh.id(e, PE) == edge_index);
    test_collapse(mesh, e, should_succeed);
}
} // namespace

TEST_CASE("trimesh_split_collapse_factories", "[operations][2D]")
{
    SECTION("split")
    {
        DEBUG_TriMesh m;
        m = single_triangle();
        auto e = m.tuple_from_id(PE, 0);

        REQUIRE(m.id(e, PE) == 0);
        REQUIRE(m.id(e, PV) == 0);
        REQUIRE(m.id(m.switch_tuple(e, PV), PV) == 1);

        test_split(m, e, true);
    }
    SECTION("collapse")
    {
        DEBUG_TriMesh m;
        m = quad();
        {
            auto e = m.tuple_from_id(PE, 0);

            REQUIRE(m.id(e, PE) == 0);
            REQUIRE(m.id(e, PV) == 0);
            REQUIRE(m.id(m.switch_tuple(e, PV), PV) == 1);

            test_collapse(m, e, false);
        }
        {
            auto e = m.tuple_from_id(PE, 0);

            REQUIRE(m.id(e, PE) == 0);
            REQUIRE(m.id(e, PV) == 0);
            REQUIRE(m.id(m.switch_tuple(e, PV), PV) == 1);

            test_collapse(m, e, false);
        }
    }
}

/*
TEST_CASE("get per face data")
{
    SECTION("single face")
    {
        DEBUG_TriMesh m = ;
        {
            //         0
            //        / \   .
            //       2   1  \ .
            //      /  0  \  \|
            //     /       \ .
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(1, 3);
            tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(0, 2, 0);
        REQUIRE(m.id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m.id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m.id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        auto state = m.get_tmoe();

        TMOE::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 1);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarGlobalIDs ear1 = face_data.ears[0];
        TMOE::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == -1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }
    SECTION("one ear")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ .
            //   2 f1 /2   1
            //   |  0/ f0  \ .
            //   |  /       \ .
            //  1  ----0---- 2
            //
            RowVectors3l tris;
            tris.resize(2, 3);
            tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<int64_t, 3, 1>{3, 1, 0};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto state = m.get_tmoe();
        TMOE::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 0);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarGlobalIDs ear1 = face_data.ears[0];
        TMOE::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }

    SECTION("two ears")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0 --1- 4
            //   |     / \     |
            //   2 f1 /2 1\ f2 |
            //   |  0/ f0  \1  0
            //   |  /       \  |
            //   1  ----0----  2
            //
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<int64_t, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<int64_t, 3, 1>{0, 2, 4};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto state = m.get_tmoe();
        TMOE::PerFaceData face_data = state.get_per_face_data(edge);
        REQUIRE(face_data.V_C_id == 0);
        REQUIRE(face_data.deleted_fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarGlobalIDs ear1 = face_data.ears[0];
        TMOE::EarGlobalIDs ear2 = face_data.ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == 2);
        REQUIRE(ear2.eid > -1);
    }
}
*/
