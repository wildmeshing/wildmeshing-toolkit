#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMesh_operations.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"


using namespace wmtk;
using namespace wmtk::tests;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe());

TEST_CASE("get per face data")
{
    SECTION("single face")
    {
        DEBUG_TriMesh m;
        {
            //         0
            //        / \   .
            //       2   1  \ .
            //      /  0  \  \|
            //     /       \ .
            //  1  ----0---- 2
            //

            m = single_triangle();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
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
            m = one_ear();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
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
            m = two_neighbors();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
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

TEST_CASE("delete simplices")
{
    // things can be marked as deleted but will still have the connectivity information
    DEBUG_TriMesh m;
    {
        m = two_neighbors();
    }
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
    std::vector<std::vector<long>> simplices_to_delete(3);
    simplices_to_delete[1] = std::vector<long>{m._debug_id(edge, PrimitiveType::Edge)};
    simplices_to_delete[2] = std::vector<long>{m._debug_id(edge, PrimitiveType::Face)};


    auto state = m.get_tmoe();
    state.simplices_to_delete = simplices_to_delete;
    state.delete_simplices();
    REQUIRE(state.flag_accessors[1].scalar_attribute(edge) == 0);
    REQUIRE(state.flag_accessors[2].scalar_attribute(edge) == 0);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[0] == -1);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[1] == 2);
    REQUIRE(state.ff_accessor.vector_attribute(edge)[2] == 1);
    REQUIRE(state.ef_accessor.scalar_attribute(edge) == 0);
}

TEST_CASE("operation state")
{
    SECTION("single face")
    {
        DEBUG_TriMesh m;
        {
            m = single_triangle();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        auto state = m.get_tmoe(edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 0);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 1);
    }
    SECTION("one ear")
    {
        DEBUG_TriMesh m;
        {
            m = one_ear();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 1);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 1);

        REQUIRE(state.FaceDatas[0].ears.size() == 2);
    }
    SECTION("interior edge")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ .
            //   2 f1 /2   1
            //   |  0/ f0  \ .
            //   |  /       \ .
            //  1  ----0---- 2
            //     \        /
            //      \  f2  /
            //       \    /
            //        \  /
            //         4
            m = interior_edge();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        REQUIRE(state.flag_accessors.size() == 3);
        REQUIRE(state.end_point_vids.size() == 2);
        REQUIRE(state.end_point_vids[0] == 1);
        REQUIRE(state.end_point_vids[1] == 2);
        REQUIRE(state.E_AB_id == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.FaceDatas.size() == 2);

        REQUIRE(state.FaceDatas[0].V_C_id == 0);
        REQUIRE(state.FaceDatas[0].deleted_fid == 0);
        REQUIRE(state.FaceDatas[0].ears.size() == 2);
        TMOE::EarGlobalIDs ear1 = state.FaceDatas[0].ears[0];
        TMOE::EarGlobalIDs ear2 = state.FaceDatas[0].ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);

        REQUIRE(state.FaceDatas[1].V_C_id == 4);
        REQUIRE(state.FaceDatas[1].deleted_fid == 2);
        REQUIRE(state.FaceDatas[1].ears.size() == 2);
        ear1 = state.FaceDatas[1].ears[0];
        ear2 = state.FaceDatas[1].ears[1];
        REQUIRE(ear1.fid == -1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }
}
TEST_CASE("glue ear to face")
{
    DEBUG_TriMesh m;
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /  .
        //    7---8
        m = hex_plus_two();
    }
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    Tuple left_ear_edge = m.switch_tuple(edge, PrimitiveType::Edge);
    REQUIRE(m._debug_id(left_ear_edge, PrimitiveType::Vertex) == 4);
    REQUIRE(
        m._debug_id(m.switch_tuple(left_ear_edge, PrimitiveType::Vertex), PrimitiveType::Vertex) ==
        1);
    auto state = m.get_tmoe();
    auto ff_accessor_before = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));
    REQUIRE(ff_accessor_before.vector_attribute(1)(2) == 2);
    state.glue_ear_to_face(1, 3, 2, m._debug_id(edge, PrimitiveType::Edge));
    auto ff_accessor_after = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));
    REQUIRE(ff_accessor_after.vector_attribute(1)(2) == 3);
}
TEST_CASE("hash update") {}

//////////// SPLIT TESTS ////////////
TEST_CASE("glue new faces across AB")
{
    // test the assumption of correct orientation
    // new face correspondance accross AB
    SECTION("single face")
    {
        // when the edge is on the boundary (indcated by FaceDatas size), there is no glue
        // across AB
        DEBUG_TriMesh m;
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
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Vertex) == 0);
        REQUIRE(m._debug_id(edge, PrimitiveType::Face) == 0);
        REQUIRE(
            m._debug_id(m.switch_tuple(edge, PrimitiveType::Vertex), PrimitiveType::Vertex) == 2);
        auto state = m.get_tmoe(edge);
        REQUIRE(state.FaceDatas.size() == 1);
    }
    SECTION("interior edge")
    {
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ .
            //   2 f1 /2   1
            //   |  0/ f0  \ .
            //   |  /  0    \ .
            //  1  --------- 2
            //     \   1    /
            //      2  f2  0
            //       \    /
            //        \  /
            //         4
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        m.reserve_attributes(PrimitiveType::Face, 10);
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        REQUIRE(state.FaceDatas.size() == 2);

        auto new_fids = state.request_simplex_indices(PrimitiveType::Face, 4);
        std::array<long, 2> new_fids_top = {new_fids[0], new_fids[1]};
        std::array<long, 2> new_fids_bottom = {new_fids[2], new_fids[3]};
        state.glue_new_faces_across_AB(new_fids_top, new_fids_bottom);


        long local_eid_top = 0;
        long local_eid_bottom = 1;

        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));

        REQUIRE(ff_accessor.vector_attribute(new_fids_top[0])[local_eid_top] == new_fids_bottom[0]);

        REQUIRE(ff_accessor.vector_attribute(new_fids_top[1])[local_eid_top] == new_fids_bottom[1]);

        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[0])[local_eid_bottom] == new_fids_top[0]);

        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[1])[local_eid_bottom] == new_fids_top[1]);
    }
}

TEST_CASE("glue new triangle", "[old faces not recycled]")
{
    SECTION("boundary edge")
    {
        DEBUG_TriMesh m;
        {
            //         0
            //        / \ 
            //       /2   1
            //      / f0  \ 
            //     /  0    \ 
            //  1  --------- 2

            RowVectors3l tris;
            tris.resize(1, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        //  create new vertex
        std::vector<long> new_vids = state.request_simplex_indices(PrimitiveType::Vertex, 1);
        REQUIRE(new_vids.size() == 1);
        const long new_vid = new_vids[0];

        // create new edges
        std::vector<long> replacement_eids = state.request_simplex_indices(PrimitiveType::Edge, 2);
        REQUIRE(replacement_eids.size() == 2);

        std::vector<std::array<long, 2>> new_fids;
        REQUIRE(state.FaceDatas.size() == 1);
        for (size_t i = 0; i < state.FaceDatas.size(); ++i) {
            const auto& face_data = state.FaceDatas[i];
            // glue the topology
            std::array<long, 2> new_fid_per_face =
                state.glue_new_triangle_topology(new_vid, replacement_eids, face_data);
            new_fids.emplace_back(new_fid_per_face);
        }
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[1] == 1);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[2] == new_vid);

        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[1] == new_vid);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[2] == 2);

        // the new fids generated are in top-down left-right order
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));

        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[1] == new_fids[0][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[2] == new_fids[0][0]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[0] == -1);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[0] == -1);

        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Edge));

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[0] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[1] == 5);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[2] == state.FaceDatas[0].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[0] == replacement_eids[1]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[1] == state.FaceDatas[0].ears[1].eid);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[2] == 5);

        auto vf_accessor = m.create_base_accessor<long>(m.vf_handle());
        REQUIRE(vf_accessor.scalar_attribute(new_vid) == new_fids[0][0]);
        REQUIRE(vf_accessor.scalar_attribute(0) == new_fids[0][0]);
        REQUIRE(vf_accessor.scalar_attribute(1) == new_fids[0][0]);
        REQUIRE(vf_accessor.scalar_attribute(2) == new_fids[0][1]);

        auto ef_accessor = m.create_base_accessor<long>(m.ef_handle());
        REQUIRE(ef_accessor.scalar_attribute(replacement_eids[0]) == new_fids[0][0]);
        REQUIRE(ef_accessor.scalar_attribute(replacement_eids[1]) == new_fids[0][1]);
        REQUIRE(ef_accessor.scalar_attribute(5) == new_fids[0][0]);
    }
    SECTION("interior edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ .
            //   2 f1 /2   1
            //   |  0/ f0  \ .
            //   |  /  0    \ .
            //  1  -------- 2
            //     \   1    /
            //      \  f2  /
            //       2    0
            //        \  /
            //         4
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        // create new vertex
        std::vector<long> new_vids = state.request_simplex_indices(PrimitiveType::Vertex, 1);
        REQUIRE(new_vids.size() == 1);
        const long new_vid = new_vids[0];

        // create new edges
        std::vector<long> replacement_eids = state.request_simplex_indices(PrimitiveType::Edge, 2);
        REQUIRE(replacement_eids.size() == 2);

        std::vector<std::array<long, 2>> new_fids;
        for (size_t i = 0; i < state.FaceDatas.size(); ++i) {
            const auto& face_data = state.FaceDatas[i];
            // glue the topology
            std::array<long, 2> new_fid_per_face =
                state.glue_new_triangle_topology(new_vid, replacement_eids, face_data);
            new_fids.emplace_back(new_fid_per_face);
        }
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[1] == 1);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[2] == new_vid);

        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[1] == new_vid);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[2] == 2);

        REQUIRE(fv_accessor.vector_attribute(new_fids[1][0])[0] == 1);
        REQUIRE(fv_accessor.vector_attribute(new_fids[1][0])[1] == 4);
        REQUIRE(fv_accessor.vector_attribute(new_fids[1][0])[2] == new_vid);

        REQUIRE(fv_accessor.vector_attribute(new_fids[1][1])[0] == new_vid);
        REQUIRE(fv_accessor.vector_attribute(new_fids[1][1])[1] == 4);
        REQUIRE(fv_accessor.vector_attribute(new_fids[1][1])[2] == 2);

        // the new fids generated are in top-down left-right order
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Face));

        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[1] == new_fids[0][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[2] == new_fids[0][0]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[1][0])[0] == new_fids[1][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[1][1])[2] == new_fids[1][0]);

        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Edge));

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[0] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[1] == 9);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[2] == state.FaceDatas[0].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[0] == replacement_eids[1]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[1] == state.FaceDatas[0].ears[1].eid);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[2] == 9);

        REQUIRE(fe_accessor.vector_attribute(new_fids[1][0])[1] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[1][0])[0] == 10);
        REQUIRE(fe_accessor.vector_attribute(new_fids[1][0])[2] == state.FaceDatas[1].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[1][1])[1] == replacement_eids[1]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[1][1])[0] == state.FaceDatas[1].ears[1].eid);
        REQUIRE(fe_accessor.vector_attribute(new_fids[1][1])[2] == 10);

        auto vf_accessor = m.create_base_accessor<long>(m.vf_handle());
        REQUIRE(vf_accessor.scalar_attribute(new_vid) == new_fids[1][0]);
        REQUIRE(vf_accessor.scalar_attribute(0) == new_fids[0][0]);
        REQUIRE(vf_accessor.scalar_attribute(4) == new_fids[1][0]);
        REQUIRE(vf_accessor.scalar_attribute(1) == new_fids[1][0]);
        REQUIRE(vf_accessor.scalar_attribute(2) == new_fids[1][1]);

        auto ef_accessor = m.create_base_accessor<long>(m.ef_handle());
        REQUIRE(ef_accessor.scalar_attribute(replacement_eids[0]) == new_fids[1][0]);
        REQUIRE(ef_accessor.scalar_attribute(replacement_eids[1]) == new_fids[1][1]);
        REQUIRE(ef_accessor.scalar_attribute(9) == new_fids[0][0]);
        REQUIRE(ef_accessor.scalar_attribute(10) == new_fids[1][0]);
    }
}

TEST_CASE("simplices to delete for split")
{
    SECTION("boundary edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m;
        {
            //         0
            //        / \ 
            //       /2   1
            //      / f0  \ 
            //     /  0    \ 
            //  1  --------- 2

            m = single_triangle();
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        state.split_edge();

        REQUIRE(state.simplices_to_delete.size() == 3);
        REQUIRE(state.simplices_to_delete[0].size() == 0);

        REQUIRE(state.simplices_to_delete[1].size() == 1);
        REQUIRE(state.simplices_to_delete[1][0] == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.simplices_to_delete[2].size() == 1);
        REQUIRE(state.simplices_to_delete[2][0] == 0);
    }
    SECTION("interior edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m;
        {
            //  3--1--- 0
            //   |     / \ 
            //   2 f1 /2   1
            //   |  0/ f0  \ 
            //   |  /  0    \ 
            //  1  -------- 2
            //     \   1    /
            //      \  f2  /
            //       2    0
            //        \  /
            //         4
            RowVectors3l tris;
            tris.resize(3, 3);
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto state = m.get_tmoe(edge);

        state.split_edge();

        REQUIRE(state.simplices_to_delete.size() == 3);
        REQUIRE(state.simplices_to_delete[0].size() == 0);

        REQUIRE(state.simplices_to_delete[1].size() == 1);
        REQUIRE(state.simplices_to_delete[1][0] == m._debug_id(edge, PrimitiveType::Edge));
        REQUIRE(state.simplices_to_delete[2].size() == 2);
        REQUIRE(state.simplices_to_delete[2][0] == 0);
        REQUIRE(state.simplices_to_delete[2][1] == 2);
    }
}

TEST_CASE("split edge")
{
    DEBUG_TriMesh m;
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ /
        //    7---8
        m = hex_plus_two();
    }
    REQUIRE(m.is_connectivity_valid());

    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    m.split_edge(edge);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge2 = m.edge_tuple_between_v1_v2(3, 0, 0);
    m.split_edge(edge2);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge3 = m.edge_tuple_between_v1_v2(4, 7, 6);
    m.split_edge(edge3);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge4 = m.edge_tuple_between_v1_v2(4, 9, 8);
    m.split_edge(edge4);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge5 = m.edge_tuple_between_v1_v2(5, 6, 4);
    m.split_edge(edge5);
    REQUIRE(m.is_connectivity_valid());
}

//////////// COLLAPSE TESTS ////////////
TEST_CASE("2D link condition for collapse") {}

TEST_CASE("collapse edge")
{
    SECTION("case1")
    {
        DEBUG_TriMesh m;
        {
            //    0---1---2
            //   / \ / \ / \
            //  3---4---5---6
            //   \ / \ /
            //    7---8
            RowVectors3l tris;
            tris.resize(8, 3);
            tris << 3, 4, 0, 4, 1, 0, 4, 5, 1, 5, 2, 1, 5, 6, 2, 3, 7, 4, 7, 8, 4, 4, 8, 5;
            m.initialize(tris);
        }
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        m.collapse_edge(edge);
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        auto state = m.get_tmoe();

        REQUIRE(state.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(2)) == 0);
        REQUIRE(state.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(7)) == 0);
        REQUIRE(fv_accessor.vector_attribute(3)(0) == 4);
        REQUIRE(fv_accessor.vector_attribute(4)(0) == 4);
    }
    SECTION("case2")
    {
        DEBUG_TriMesh m;
        {
            //    0---1---2
            //   / \ / \ / \
            //  3---4---5---6
            //   \ / \ /
            //    7---8
            RowVectors3l tris;
            tris.resize(8, 3);
            tris << 3, 4, 0, 4, 1, 0, 4, 5, 1, 5, 2, 1, 5, 6, 2, 3, 7, 4, 7, 8, 4, 4, 8, 5;
            m.initialize(tris);
        }
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);
        m.collapse_edge(edge);
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        auto state = m.get_tmoe();

        REQUIRE(state.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(0)) == 0);
        REQUIRE(state.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);
        REQUIRE(fv_accessor.vector_attribute(2)(0) == 0);
        REQUIRE(fv_accessor.vector_attribute(5)(2) == 0);
        REQUIRE(fv_accessor.vector_attribute(6)(2) == 0);
        REQUIRE(fv_accessor.vector_attribute(7)(0) == 0);
    }
}
