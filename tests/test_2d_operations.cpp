#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/TriMeshSwapEdgeOperation.hpp>
#include <wmtk/operations/TriMeshCollapseEdgeOperation.hpp>

using namespace wmtk;
using namespace wmtk::tests;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(wmtk::Tuple()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;

TEST_CASE("incident_face_data", "[operations][2D]")
{
    SECTION("single_face")
    {
        //         0
        //        / \   .
        //       2   1  \ .
        //      /  0  \  \|
        //     /       \ .
        //  1  ----0---- 2
        //
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PV) == 0);
        REQUIRE(m._debug_id(edge, PF) == 0);
        REQUIRE(m._debug_id(m.switch_tuple(edge, PV), PV) == 2);
        auto executor = m.get_tmoe(edge);

        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 1);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarFace ear1 = face_data.ears[0];
        TMOE::EarFace ear2 = face_data.ears[1];
        CHECK(ear1.fid == -1);
        CHECK(ear1.eid > -1);
        CHECK(ear2.fid == -1);
        CHECK(ear2.eid > -1);
    }
    SECTION("one_ear")
    {
        //  3--1--- 0
        //   |     / \ .
        //   2 f1 /2   1
        //   |  0/ f0  \ .
        //   |  /       \ .
        //  1  ----0---- 2
        //
        DEBUG_TriMesh m = one_ear();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        const auto executor = m.get_tmoe(edge);
        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 0);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarFace ear1 = face_data.ears[0];
        TMOE::EarFace ear2 = face_data.ears[1];
        CHECK(ear1.fid == 1);
        CHECK(ear1.eid > -1);
        CHECK(ear2.fid == -1);
        CHECK(ear2.eid > -1);
    }
    SECTION("two_ears")
    {
        //  3--1--- 0 --1- 4
        //   |     / \     |
        //   2 f1 /2 1\ f2 |
        //   |  0/ f0  \1  0
        //   |  /       \  |
        //   1  ----0----  2
        //
        DEBUG_TriMesh m = two_neighbors();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);
        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 0);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarFace ear1 = face_data.ears[0];
        TMOE::EarFace ear2 = face_data.ears[1];
        CHECK(ear1.fid == 1);
        CHECK(ear1.eid > -1);
        CHECK(ear2.fid == 2);
        CHECK(ear2.eid > -1);
    }
}

TEST_CASE("delete_simplices", "[operations][2D]")
{
    // things can be marked as deleted but will still have the connectivity information
    DEBUG_TriMesh m = two_neighbors();
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
    std::vector<std::vector<long>> simplices_to_delete(3);
    simplices_to_delete[1] = std::vector<long>{m._debug_id(edge, PE)};
    simplices_to_delete[2] = std::vector<long>{m._debug_id(edge, PF)};


    auto executor = m.get_tmoe(edge);
    executor.simplices_to_delete = simplices_to_delete;
    executor.delete_simplices();
    REQUIRE(executor.flag_accessors[1].scalar_attribute(edge) == 0);
    REQUIRE(executor.flag_accessors[2].scalar_attribute(edge) == 0);
    REQUIRE(executor.ff_accessor.vector_attribute(edge)[0] == -1);
    REQUIRE(executor.ff_accessor.vector_attribute(edge)[1] == 2);
    REQUIRE(executor.ff_accessor.vector_attribute(edge)[2] == 1);
    REQUIRE(executor.ef_accessor.scalar_attribute(edge) == 0);
}

TEST_CASE("operation_state", "[operations][2D]")
{
    SECTION("single_face")
    {
        DEBUG_TriMesh m = single_triangle();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PV) == 0);
        REQUIRE(m._debug_id(edge, PF) == 0);
        REQUIRE(m._debug_id(m.switch_tuple(edge, PV), PV) == 2);
        auto executor = m.get_tmoe(edge);

        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 0);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m._debug_id(edge, PE));
        REQUIRE(executor.incident_face_datas().size() == 1);
    }
    SECTION("one_ear")
    {
        DEBUG_TriMesh m = one_ear();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 1);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m._debug_id(edge, PE));
        REQUIRE(executor.incident_face_datas().size() == 1);

        REQUIRE(executor.incident_face_datas()[0].ears.size() == 2);
    }
    SECTION("interior_edge")
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
        DEBUG_TriMesh m = interior_edge();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 1);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m._debug_id(edge, PE));
        REQUIRE(executor.incident_face_datas().size() == 2);

        REQUIRE(executor.incident_face_datas()[0].opposite_vid == 0);
        REQUIRE(executor.incident_face_datas()[0].fid == 0);
        REQUIRE(executor.incident_face_datas()[0].ears.size() == 2);
        TMOE::EarFace ear1 = executor.incident_face_datas()[0].ears[0];
        TMOE::EarFace ear2 = executor.incident_face_datas()[0].ears[1];
        REQUIRE(ear1.fid == 1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);

        REQUIRE(executor.incident_face_datas()[1].opposite_vid == 4);
        REQUIRE(executor.incident_face_datas()[1].fid == 2);
        REQUIRE(executor.incident_face_datas()[1].ears.size() == 2);
        ear1 = executor.incident_face_datas()[1].ears[0];
        ear2 = executor.incident_face_datas()[1].ears[1];
        REQUIRE(ear1.fid == -1);
        REQUIRE(ear1.eid > -1);
        REQUIRE(ear2.fid == -1);
        REQUIRE(ear2.eid > -1);
    }
}
TEST_CASE("glue_ear_to_face", "[operations][2D]")
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
    //    7---8
    DEBUG_TriMesh m = hex_plus_two();

    REQUIRE(m.is_connectivity_valid());
    const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    const Tuple left_ear_edge = m.switch_tuple(edge, PE);
    REQUIRE(m._debug_id(left_ear_edge, PV) == 4);
    REQUIRE(m._debug_id(m.switch_tuple(left_ear_edge, PV), PV) == 1);
    auto executor = m.get_tmoe(edge);
    auto ff_accessor_before = m.create_base_accessor<long>(m.f_handle(PF));
    REQUIRE(ff_accessor_before.vector_attribute(1)(2) == 2);
    executor.glue_ear_to_face(1, 3, 2, m._debug_id(edge, PE));
    auto ff_accessor_after = m.create_base_accessor<long>(m.f_handle(PF));
    REQUIRE(ff_accessor_after.vector_attribute(1)(2) == 3);
}
TEST_CASE("hash_update", "[operations][2D][.]")
{
    REQUIRE(false);
}

//////////// SPLIT TESTS ////////////
TEST_CASE("glue_new_faces_across_AB", "[operations][2D]")
{
    // test the assumption of correct orientation
    // new face correspondance accross AB
    SECTION("single_face")
    {
        // when the edge is on the boundary (indcated by FaceDatas size), there is no glue
        // across AB
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);
        REQUIRE(m._debug_id(edge, PV) == 0);
        REQUIRE(m._debug_id(edge, PF) == 0);
        REQUIRE(m._debug_id(m.switch_tuple(edge, PV), PV) == 2);
        auto executor = m.get_tmoe(edge);
        REQUIRE(executor.incident_face_datas().size() == 1);
    }
    SECTION("interior_edge")
    {
        DEBUG_TriMesh m = interior_edge();
        m.reserve_attributes(PF, 10);
        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        REQUIRE(executor.incident_face_datas().size() == 2);

        const auto new_fids = executor.request_simplex_indices(PF, 4);
        const std::array<long, 2> new_fids_top = {new_fids[0], new_fids[1]};
        const std::array<long, 2> new_fids_bottom = {new_fids[2], new_fids[3]};
        executor.glue_new_faces_across_AB(new_fids_top, new_fids_bottom);


        long local_eid_top = 0;
        long local_eid_bottom = 1;

        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));

        REQUIRE(ff_accessor.vector_attribute(new_fids_top[0])[local_eid_top] == new_fids_bottom[0]);

        REQUIRE(ff_accessor.vector_attribute(new_fids_top[1])[local_eid_top] == new_fids_bottom[1]);

        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[0])[local_eid_bottom] == new_fids_top[0]);

        REQUIRE(
            ff_accessor.vector_attribute(new_fids_bottom[1])[local_eid_bottom] == new_fids_top[1]);
    }
}

TEST_CASE("glue_new_triangle", "[operations][2D]")
{
    SECTION("boundary_edge")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        //  create new vertex
        std::vector<long> new_vids = executor.request_simplex_indices(PV, 1);
        REQUIRE(new_vids.size() == 1);
        const long new_vid = new_vids[0];

        // create new edges
        std::vector<long> replacement_eids = executor.request_simplex_indices(PE, 2);
        REQUIRE(replacement_eids.size() == 2);

        std::vector<std::array<long, 2>> new_fids;
        REQUIRE(executor.incident_face_datas().size() == 1);
        for (size_t i = 0; i < executor.incident_face_datas().size(); ++i) {
            const auto& face_data = executor.incident_face_datas()[i];
            // glue the topology
            std::array<long, 2> new_fid_per_face =
                executor.glue_new_triangle_topology(new_vid, replacement_eids, face_data);
            new_fids.emplace_back(new_fid_per_face);
        }
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[1] == 1);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][0])[2] == new_vid);

        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[0] == 0);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[1] == new_vid);
        REQUIRE(fv_accessor.vector_attribute(new_fids[0][1])[2] == 2);

        // the new fids generated are in top-down left-right order
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));

        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[1] == new_fids[0][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[2] == new_fids[0][0]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[0] == -1);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[0] == -1);

        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PE));

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[0] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[1] == 5);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[0][0])[2] ==
            executor.incident_face_datas()[0].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[0] == replacement_eids[1]);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[0][1])[1] ==
            executor.incident_face_datas()[0].ears[1].eid);
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
    SECTION("interior_edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m = interior_edge();
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        // create new vertex
        std::vector<long> new_vids = executor.request_simplex_indices(PV, 1);
        REQUIRE(new_vids.size() == 1);
        const long new_vid = new_vids[0];

        // create new edges
        std::vector<long> replacement_eids = executor.request_simplex_indices(PE, 2);
        REQUIRE(replacement_eids.size() == 2);

        std::vector<std::array<long, 2>> new_fids;
        for (size_t i = 0; i < executor.incident_face_datas().size(); ++i) {
            const auto& face_data = executor.incident_face_datas()[i];
            // glue the topology
            std::array<long, 2> new_fid_per_face =
                executor.glue_new_triangle_topology(new_vid, replacement_eids, face_data);
            new_fids.emplace_back(new_fid_per_face);
        }
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));
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
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));

        REQUIRE(ff_accessor.vector_attribute(new_fids[0][0])[1] == new_fids[0][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[0][1])[2] == new_fids[0][0]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[1][0])[0] == new_fids[1][1]);
        REQUIRE(ff_accessor.vector_attribute(new_fids[1][1])[2] == new_fids[1][0]);

        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PE));

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[0] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][0])[1] == 9);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[0][0])[2] ==
            executor.incident_face_datas()[0].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[0] == replacement_eids[1]);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[0][1])[1] ==
            executor.incident_face_datas()[0].ears[1].eid);
        REQUIRE(fe_accessor.vector_attribute(new_fids[0][1])[2] == 9);

        REQUIRE(fe_accessor.vector_attribute(new_fids[1][0])[1] == replacement_eids[0]);
        REQUIRE(fe_accessor.vector_attribute(new_fids[1][0])[0] == 10);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[1][0])[2] ==
            executor.incident_face_datas()[1].ears[0].eid);

        REQUIRE(fe_accessor.vector_attribute(new_fids[1][1])[1] == replacement_eids[1]);
        REQUIRE(
            fe_accessor.vector_attribute(new_fids[1][1])[0] ==
            executor.incident_face_datas()[1].ears[1].eid);
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

TEST_CASE("simplices_to_delete_for_split", "[operations][2D]")
{
    SECTION("boundary_edge")
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
        auto executor = m.get_tmoe(edge);

        executor.split_edge();

        REQUIRE(executor.simplices_to_delete.size() == 3);
        REQUIRE(executor.simplices_to_delete[0].size() == 0);

        REQUIRE(executor.simplices_to_delete[1].size() == 1);
        REQUIRE(executor.simplices_to_delete[1][0] == m._debug_id(edge, PE));
        REQUIRE(executor.simplices_to_delete[2].size() == 1);
        REQUIRE(executor.simplices_to_delete[2][0] == 0);
    }
    SECTION("interior_edge")
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
        auto executor = m.get_tmoe(edge);

        executor.split_edge();

        REQUIRE(executor.simplices_to_delete.size() == 3);
        REQUIRE(executor.simplices_to_delete[0].size() == 0);

        REQUIRE(executor.simplices_to_delete[1].size() == 1);
        REQUIRE(executor.simplices_to_delete[1][0] == m._debug_id(edge, PE));
        REQUIRE(executor.simplices_to_delete[2].size() == 2);
        REQUIRE(executor.simplices_to_delete[2][0] == 0);
        REQUIRE(executor.simplices_to_delete[2][1] == 2);
    }
}

TEST_CASE("split_edge", "[operations][2D]")
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    DEBUG_TriMesh m = hex_plus_two();

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

TEST_CASE("collapse_edge", "[operations][2D]")
{
    DEBUG_TriMesh m = hex_plus_two();
    SECTION("case1")
    {
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        auto executor = m.get_tmoe(edge);
        m.collapse_edge(edge);
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(2)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(7)) == 0);
        REQUIRE(fv_accessor.vector_attribute(0)(1) == 9);
        REQUIRE(fv_accessor.vector_attribute(1)(0) == 9);
        REQUIRE(fv_accessor.vector_attribute(3)(0) == 9);
        REQUIRE(fv_accessor.vector_attribute(5)(2) == 9);
        REQUIRE(fv_accessor.vector_attribute(6)(2) == 9);
        REQUIRE(fv_accessor.vector_attribute(4)(0) == 9);
    }
    SECTION("case2")
    {
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);
        auto executor = m.get_tmoe(edge);
        m.collapse_edge(edge);
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(0)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        REQUIRE(fv_accessor.vector_attribute(2)(0) == 9);
        REQUIRE(fv_accessor.vector_attribute(5)(2) == 9);
        REQUIRE(fv_accessor.vector_attribute(6)(2) == 9);
        REQUIRE(fv_accessor.vector_attribute(7)(0) == 9);
    }
    SECTION("test return tuple")
    {
        DEBUG_TriMesh m = hex_plus_two();
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(3, 4, 0);
        TriMeshCollapseEdgeOperation op(m,edge);
        op();
        auto ret = op.return_tuple();
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(op.is_return_tuple_from_left_ear() == false);
        REQUIRE(m.id(ret, PV) == 9);
        REQUIRE(m.id(m.switch_tuple(ret, PV), PV) == 1);
    }

    SECTION("test return tuple 2")
    {
        DEBUG_TriMesh m = hex_plus_two();
        std::cout << "BEFORE COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(4, 3, 0);
        TriMeshCollapseEdgeOperation op(m,edge);
        op();
        auto ret = op.return_tuple();
        std::cout << "AFTER COLLAPSE" << std::endl;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(op.is_return_tuple_from_left_ear() == true);
        REQUIRE(m.id(ret, PV) == 9);
        REQUIRE(m.id(m.switch_tuple(ret, PV), PV) == 1);
    }
}

TEST_CASE("swap_edge", "[operations][2D]")
{
    SECTION("case ccw")
    {
        DEBUG_TriMesh m = hex_plus_two();
        std::cout << "BEFORE SWAP" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        TriMeshSwapEdgeOperation op(m,edge);
        op();
        auto ret = op.return_tuple();
        std::cout << "AFTER SWAP" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        std::cout << m.id(ret,PV) << "," << m.id(m.switch_tuple(ret, PV), PV) << std::endl;
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        
        REQUIRE(m.id(ret,PV) == 8);
        REQUIRE(m.id(m.switch_tuple(ret, PV), PV) == 10);
        REQUIRE(fv_accessor.vector_attribute(10)(0) == 4);
        REQUIRE(fv_accessor.vector_attribute(10)(1) == 8);
        REQUIRE(fv_accessor.vector_attribute(10)(2) == 10);
        REQUIRE(fv_accessor.vector_attribute(11)(0) == 10);
        REQUIRE(fv_accessor.vector_attribute(11)(1) == 8);
        REQUIRE(fv_accessor.vector_attribute(11)(2) == 5);
    }

    SECTION("case cw")
    {
        DEBUG_TriMesh m = hex_plus_two();
        std::cout << "BEFORE SWAP" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        Tuple edge = m.edge_tuple_between_v1_v2(5, 4, 2);
        auto executor = m.get_tmoe(edge);
        TriMeshSwapEdgeOperation op(m,edge);
        op();
        auto ret = op.return_tuple();
        std::cout << "AFTER SWAP" << std::endl;
        REQUIRE(m.is_connectivity_valid());

        std::cout << m.id(ret,PV) << "," << m.id(m.switch_tuple(ret, PV), PV) << std::endl;
        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        
        // for (long i = 0; i < m.capacity(PF); i++)
        // {
        //     if (executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(i)) == 0) continue;
        //     std::cout << fv_accessor.vector_attribute(i)(0) << " " << fv_accessor.vector_attribute(i)(1) << " " << fv_accessor.vector_attribute(i)(2) << std::endl;
        // }
        REQUIRE(m.id(ret,PV) == 10);
        REQUIRE(m.id(m.switch_tuple(ret, PV), PV) == 8);
        REQUIRE(fv_accessor.vector_attribute(11)(0) == 4);
        REQUIRE(fv_accessor.vector_attribute(11)(1) == 8);
        REQUIRE(fv_accessor.vector_attribute(11)(2) == 10);
        REQUIRE(fv_accessor.vector_attribute(10)(0) == 10);
        REQUIRE(fv_accessor.vector_attribute(10)(1) == 8);
        REQUIRE(fv_accessor.vector_attribute(10)(2) == 5);
    }
}
