#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <numeric>
#include <set>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
#include "../tools/is_free.hpp"
#include "../tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::tests;
using namespace operations;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<int64_t, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(wmtk::Tuple()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;

TEST_CASE("get_split_simplices_to_delete", "[operations][split][2D]")
{
    SECTION("single_triangle")
    {
        const DEBUG_TriMesh m = single_triangle();
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);

        std::array<std::vector<int64_t>, 3> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 1);

        const int64_t edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m.id(edge, PE));
        const int64_t face_to_delete = ids_to_delete[2][0];
        CHECK(face_to_delete == m.id(edge, PF));
    }
    SECTION("hex_plus_two")
    {
        const DEBUG_TriMesh m = hex_plus_two();
        const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);

        std::array<std::vector<int64_t>, 3> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 2);

        const int64_t edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m.id(edge, PE));

        // compare expected face ids with the actual ones that should be deleted
        std::set<int64_t> fid_expected;
        fid_expected.insert(m.id(edge, PF));
        fid_expected.insert(m.id(m.switch_face(edge), PF));

        std::set<int64_t> fid_actual;
        for (const int64_t& f : ids_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
    SECTION("free")
    {
        const DEBUG_TriMesh m = []() {
            TriMesh m;
            m.initialize_free(20);
            return m;
        }();
        for (Tuple edge : m.get_all(PrimitiveType::Edge)) {
            std::array<std::vector<int64_t>, 3> ids_to_delete =
                TMOE::get_split_simplices_to_delete(edge, m);

            REQUIRE(ids_to_delete[0].size() == 0);
            REQUIRE(ids_to_delete[1].size() == 1);
            REQUIRE(ids_to_delete[2].size() == 1);

            // compare expected face ids with the actual ones that should be deleted
            std::set<int64_t> fid_expected;
            fid_expected.insert(m.id(edge, PF));

            std::set<int64_t> fid_actual;
            for (const int64_t& f : ids_to_delete[2]) {
                CHECK(fid_expected.find(f) != fid_expected.end());
                fid_actual.insert(f);
            }
            CHECK(fid_actual.size() == fid_expected.size());
        }
    }
}


TEST_CASE("delete_simplices", "[operations][2D]")
{
    // delete for split

    // things can be marked as deleted but will still have the connectivity information
    DEBUG_TriMesh m = two_neighbors();
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
    std::vector<std::vector<int64_t>> simplices_to_delete(3);
    const int64_t edge_index = m.id(edge, PE);
    const int64_t face_index = m.id(edge, PF);
    simplices_to_delete[1].emplace_back(edge_index);
    simplices_to_delete[2].emplace_back(face_index);

    auto executor = m.get_tmoe(edge);
    executor.split_edge_precompute();

    // new way of getting simplices
    executor.simplex_ids_to_delete = TMOE::get_split_simplices_to_delete(edge, m);

    executor.delete_simplices();
    REQUIRE(executor.flag_accessors[1].index_access().is_active(edge_index) == false);
    REQUIRE(executor.flag_accessors[2].index_access().is_active(face_index) == false);
    REQUIRE(executor.ff_accessor.const_vector_attribute(face_index)[0] == -1);
    REQUIRE(executor.ff_accessor.const_vector_attribute(face_index)[1] == 2);
    REQUIRE(executor.ff_accessor.const_vector_attribute(face_index)[2] == 1);
    REQUIRE(executor.ef_accessor.const_scalar_attribute(edge_index) == 0);
}

TEST_CASE("operation_state", "[operations][2D]")
{
    SECTION("single_face")
    {
        DEBUG_TriMesh m = single_triangle();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(0, 2, 0);
        REQUIRE(m.id(edge, PV) == 0);
        REQUIRE(m.id(edge, PF) == 0);
        REQUIRE(m.id(m.switch_tuple(edge, PV), PV) == 2);
        auto executor = m.get_tmoe(edge);

        executor.split_edge_precompute();
        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 0);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m.id(edge, PE));
        REQUIRE(executor.incident_face_datas().size() == 1);
    }
    SECTION("one_ear")
    {
        DEBUG_TriMesh m = one_ear();

        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        executor.split_edge_precompute();
        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 1);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m.id(edge, PE));
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
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto executor = m.get_tmoe(edge);

        executor.split_edge_precompute();
        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 1);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m.id(edge, PE));
        REQUIRE(executor.incident_face_datas().size() == 2);

        REQUIRE(executor.incident_face_datas()[0].opposite_vid == 0);
        REQUIRE(executor.incident_face_datas()[0].fid == 0);
        REQUIRE(executor.incident_face_datas()[0].ears.size() == 2);
        TMOE::EarData ear1 = executor.incident_face_datas()[0].ears[0];
        TMOE::EarData ear2 = executor.incident_face_datas()[0].ears[1];
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
    SECTION("free")
    {
        DEBUG_TriMesh m = []() {
            TriMesh m;
            m.initialize_free(20);
            return m;
        }();
        for (Tuple edge : m.get_all(PrimitiveType::Edge)) {
            REQUIRE(m.is_connectivity_valid());
            auto executor = m.get_tmoe(edge);
            auto s = m.create_scope();
            executor.split_edge_precompute();

            REQUIRE(executor.flag_accessors.size() == 3);
            REQUIRE(executor.incident_face_datas().size() == 1);

            TMOE::EarData ear1 = executor.incident_face_datas()[0].ears[0];
            TMOE::EarData ear2 = executor.incident_face_datas()[0].ears[1];
            REQUIRE(ear1.fid == -1);
            REQUIRE(ear1.eid > -1);
            REQUIRE(ear2.fid == -1);
            REQUIRE(ear2.eid > -1);
            // executor enables faces that we don't want enabled so lets deactivate them
            s.mark_failed();
        }
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
    const Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
    const Tuple left_ear_edge = m.switch_tuple(edge, PE);
    REQUIRE(m.id(left_ear_edge, PV) == 4);
    REQUIRE(m.id(m.switch_tuple(left_ear_edge, PV), PV) == 1);
    auto executor = m.get_tmoe(edge);
    executor.split_edge_precompute();
    auto& ff_accessor_before = m.create_base_accessor<int64_t>(m.f_handle(PF));
    REQUIRE(ff_accessor_before.vector_attribute(1)(2) == 2);
    TMOE::EarData ear{1, m.id(edge, PE)};
    executor.update_ids_in_ear(ear, 3, 2);
    auto& ff_accessor_after = m.create_base_accessor<int64_t>(m.f_handle(PF));
    REQUIRE(ff_accessor_after.vector_attribute(1)(2) == 3);
}


//////////// SPLIT TESTS ////////////
TEST_CASE("connect_faces_across_spine", "[operations][split][2D]")
{
    DEBUG_TriMesh m = interior_edge();
    m.reserve_attributes(PF, 10);
    REQUIRE(m.is_connectivity_valid());
    const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
    auto executor = m.get_tmoe(edge);
    executor.split_edge_precompute();
    auto& incident_face_datas = executor.m_incident_face_datas;

    REQUIRE(executor.incident_face_datas().size() == 2);

    const auto new_fids = executor.request_simplex_indices(PF, 4);
    int64_t& f0_top = incident_face_datas[0].split_f[0];
    int64_t& f1_top = incident_face_datas[0].split_f[1];
    int64_t& f0_bottom = incident_face_datas[1].split_f[0];
    int64_t& f1_bottom = incident_face_datas[1].split_f[1];
    f0_top = new_fids[0];
    f1_top = new_fids[2];
    f0_bottom = new_fids[1];
    f1_bottom = new_fids[3];

    executor.connect_faces_across_spine();

    const int64_t local_eid_top = 0;
    const int64_t local_eid_bottom = 1;

    auto& ff_accessor = m.create_base_accessor<int64_t>(m.f_handle(PF));

    CHECK(ff_accessor.vector_attribute(f0_top)[local_eid_top] == f0_bottom);
    CHECK(ff_accessor.vector_attribute(f1_top)[local_eid_top] == f1_bottom);
    CHECK(ff_accessor.vector_attribute(f0_bottom)[local_eid_bottom] == f0_top);
    CHECK(ff_accessor.vector_attribute(f1_bottom)[local_eid_bottom] == f1_top);
}

TEST_CASE("replace_incident_face", "[operations][split][2D]")
{
    SECTION("boundary_edge")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto executor = m.get_tmoe(edge);
        executor.split_edge_precompute();
        auto& incident_face_datas = executor.m_incident_face_datas;


        CHECK(executor.split_new_vid == 3);

        CHECK(executor.split_spine_eids[0] == 3);
        CHECK(executor.split_spine_eids[1] == 4);

        REQUIRE(incident_face_datas.size() == 1);


        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));
        auto& ff_accessor = m.create_base_accessor<int64_t>(m.f_handle(PF));


        std::cout << "FV: " << fv_accessor.const_vector_attribute(0).transpose() << std::endl;
        std::cout << "FF: " << ff_accessor.const_vector_attribute(0).transpose() << std::endl;


        for (size_t i = 0; i < incident_face_datas.size(); ++i) {
            auto& face_data = incident_face_datas[i];
            executor.replace_incident_face(face_data);
        }
        REQUIRE(incident_face_datas.size() == 1);

        const int64_t& f0 = incident_face_datas[0].split_f[0];
        const int64_t& f1 = incident_face_datas[0].split_f[1];
        const int64_t& se0 = executor.split_spine_eids[0];
        const int64_t& se1 = executor.split_spine_eids[1];
        const int64_t& ee0 = incident_face_datas[0].ears[0].eid;
        const int64_t& ee1 = incident_face_datas[0].ears[1].eid;

        const auto fv0 = fv_accessor.const_vector_attribute(f0);
        const auto fv1 = fv_accessor.const_vector_attribute(f1);
        CHECK(fv0[0] == 0);
        CHECK(fv0[1] == 1);
        CHECK(fv0[2] == executor.split_new_vid);

        CHECK(fv1[0] == 0);
        CHECK(fv1[1] == executor.split_new_vid);
        CHECK(fv1[2] == 2);

        // the new fids generated are in top-down left-right order
        const auto ff0 = ff_accessor.const_vector_attribute(f0);
        const auto ff1 = ff_accessor.const_vector_attribute(f1);
        CHECK(ff0[0] == -1);
        CHECK(ff0[1] == f1);
        CHECK(ff0[2] == -1);

        CHECK(ff1[0] == -1);
        CHECK(ff1[1] == -1);
        CHECK(ff1[2] == f0);

        auto& fe_accessor = m.create_base_accessor<int64_t>(m.f_handle(PE));
        const auto fe0 = fe_accessor.vector_attribute(f0);
        const auto fe1 = fe_accessor.vector_attribute(f1);

        CHECK(fe0[0] == se0);
        CHECK(fe0[1] == 5);
        CHECK(fe0[2] == ee0);

        CHECK(fe1[0] == se1);
        CHECK(fe1[1] == ee1);
        CHECK(fe1[2] == 5);

        auto& vf_accessor = m.create_base_accessor<int64_t>(m.vf_handle());
        CHECK(vf_accessor.scalar_attribute(executor.split_new_vid) == f0);
        CHECK(vf_accessor.scalar_attribute(0) == f0);
        CHECK(vf_accessor.scalar_attribute(1) == f0);
        CHECK(vf_accessor.scalar_attribute(2) == f1);

        auto& ef_accessor = m.create_base_accessor<int64_t>(m.ef_handle());
        CHECK(ef_accessor.scalar_attribute(se0) == f0);
        CHECK(ef_accessor.scalar_attribute(se1) == f1);
        CHECK(ef_accessor.scalar_attribute(ee0) == f0);
        CHECK(ef_accessor.scalar_attribute(ee1) == f1);
        CHECK(ef_accessor.scalar_attribute(5) == f0);
    }
    SECTION("interior_edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m = interior_edge();
        REQUIRE(m.is_connectivity_valid());
        Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        auto executor = m.get_tmoe(edge);
        executor.split_edge_precompute();
        auto& incident_face_datas = executor.m_incident_face_datas;

        CHECK(executor.split_new_vid == 5);

        CHECK(executor.split_spine_eids[0] == 7);
        CHECK(executor.split_spine_eids[1] == 8);


        for (size_t i = 0; i < incident_face_datas.size(); ++i) {
            auto& face_data = incident_face_datas[i];
            executor.replace_incident_face(face_data);
        }
        REQUIRE(incident_face_datas.size() == 2);

        auto& fv_accessor = m.create_base_accessor<int64_t>(m.f_handle(PV));
        auto& fe_accessor = m.create_base_accessor<int64_t>(m.f_handle(PE));
        auto& ff_accessor = m.create_base_accessor<int64_t>(m.f_handle(PF));

        auto& vf_accessor = m.create_base_accessor<int64_t>(m.vf_handle());

        auto& ef_accessor = m.create_base_accessor<int64_t>(m.ef_handle());

        const int64_t& se0 = executor.split_spine_eids[0];
        const int64_t& se1 = executor.split_spine_eids[1];

        // top
        {
            const int64_t& f0 = incident_face_datas[0].split_f[0];
            const int64_t& f1 = incident_face_datas[0].split_f[1];
            const int64_t& ee0 = incident_face_datas[0].ears[0].eid;
            const int64_t& ee1 = incident_face_datas[0].ears[1].eid;

            const auto fv0 = fv_accessor.vector_attribute(f0);
            const auto fv1 = fv_accessor.vector_attribute(f1);
            CHECK(fv0[0] == 0);
            CHECK(fv0[1] == 1);
            CHECK(fv0[2] == executor.split_new_vid);

            CHECK(fv1[0] == 0);
            CHECK(fv1[1] == executor.split_new_vid);
            CHECK(fv1[2] == 2);

            const auto ff0 = ff_accessor.vector_attribute(f0);
            const auto ff1 = ff_accessor.vector_attribute(f1);
            CHECK(ff0[0] == incident_face_datas[1].fid);
            CHECK(ff0[1] == f1);
            CHECK(ff0[2] == incident_face_datas[0].ears[0].fid);
            CHECK(ff1[0] == incident_face_datas[1].fid);
            CHECK(ff1[1] == -1);
            CHECK(ff1[2] == f0);

            const auto fe0 = fe_accessor.vector_attribute(f0);
            const auto fe1 = fe_accessor.vector_attribute(f1);
            CHECK(fe0[0] == executor.split_spine_eids[0]);
            CHECK(fe0[1] == 9);
            CHECK(fe0[2] == ee0);

            CHECK(fe1[0] == executor.split_spine_eids[1]);
            CHECK(fe1[1] == ee1);
            CHECK(fe1[2] == 9);

            CHECK(vf_accessor.scalar_attribute(0) == f0);

            CHECK(ef_accessor.scalar_attribute(ee0) == f0);
            CHECK(ef_accessor.scalar_attribute(ee1) == f1);
            CHECK(ef_accessor.scalar_attribute(9) == f0);
        }
        // bottom
        {
            const int64_t& f0 = incident_face_datas[1].split_f[0];
            const int64_t& f1 = incident_face_datas[1].split_f[1];
            const int64_t& ee0 = incident_face_datas[1].ears[0].eid;
            const int64_t& ee1 = incident_face_datas[1].ears[1].eid;

            const auto fv0 = fv_accessor.vector_attribute(f0);
            const auto fv1 = fv_accessor.vector_attribute(f1);

            CHECK(fv0[0] == 1);
            CHECK(fv0[1] == 4);
            CHECK(fv0[2] == executor.split_new_vid);

            CHECK(fv1[0] == executor.split_new_vid);
            CHECK(fv1[1] == 4);
            CHECK(fv1[2] == 2);

            const auto ff0 = ff_accessor.vector_attribute(f0);
            const auto ff1 = ff_accessor.vector_attribute(f1);
            CHECK(ff0[0] == f1);
            CHECK(ff0[1] == incident_face_datas[0].fid);
            CHECK(ff0[2] == -1);

            CHECK(ff1[0] == -1);
            CHECK(ff1[1] == incident_face_datas[0].fid);
            CHECK(ff1[2] == f0);

            const auto fe0 = fe_accessor.vector_attribute(f0);
            const auto fe1 = fe_accessor.vector_attribute(f1);
            CHECK(fe0[0] == 10);
            CHECK(fe0[1] == se0);
            CHECK(fe0[2] == ee0);

            CHECK(fe1[0] == ee1);
            CHECK(fe1[1] == se1);
            CHECK(fe1[2] == 10);

            CHECK(vf_accessor.scalar_attribute(executor.split_new_vid) == f0);
            CHECK(vf_accessor.scalar_attribute(4) == f0);
            CHECK(vf_accessor.scalar_attribute(1) == f0);
            CHECK(vf_accessor.scalar_attribute(2) == f1);

            CHECK(ef_accessor.scalar_attribute(se0) == f0);
            CHECK(ef_accessor.scalar_attribute(se1) == f1);
            CHECK(ef_accessor.scalar_attribute(ee0) == f0);
            CHECK(ef_accessor.scalar_attribute(ee1) == f1);
            CHECK(ef_accessor.scalar_attribute(10) == f0);
        }
    }
}

TEST_CASE("simplices_to_delete_for_split", "[operations][split][2D]")
{
    SECTION("boundary_edge")
    {
        // old faces are not recycled
        DEBUG_TriMesh m;
        {
            //         0
            //        / \  .
            //       /2   1
            //      / f0  \ .
            //     /  0    \  .
            //  1  --------- 2

            m = single_triangle();
        }
        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        const int64_t edge_id = m.id(edge, PE);
        auto executor = m.get_tmoe(edge);

        executor.split_edge();

        const auto& ids_to_delete = executor.simplex_ids_to_delete;
        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
        REQUIRE(ids_to_delete[2].size() == 1);
        REQUIRE(ids_to_delete[2][0] == 0);
    }
    SECTION("interior_edge")
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
            tris.row(0) = Eigen::Matrix<int64_t, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<int64_t, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<int64_t, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        const int64_t edge_id = m.id(edge, PE);
        auto executor = m.get_tmoe(edge);

        executor.split_edge();

        const auto& ids_to_delete = executor.simplex_ids_to_delete;

        REQUIRE(ids_to_delete[0].size() == 0);

        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[1][0] == edge_id);
        REQUIRE(ids_to_delete[2].size() == 2);
        REQUIRE(ids_to_delete[2][0] == 0);
        REQUIRE(ids_to_delete[2][1] == 2);
    }
}

TEST_CASE("split_edge", "[operations][split][2D]")
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /
    //    7---8
    DEBUG_TriMesh m = hex_plus_two();
    m.reserve_more_attributes({10, 10, 10});

    REQUIRE(m.is_connectivity_valid());
    EdgeSplit split(m);

    Tuple edge = m.edge_tuple_with_vs_and_t(4, 5, 2);
    split(Simplex::edge(m, edge));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge2 = m.edge_tuple_with_vs_and_t(3, 0, 0);
    split(Simplex::edge(m, edge2));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge3 = m.edge_tuple_with_vs_and_t(4, 7, 6);
    REQUIRE(m.is_valid(edge3));
    split(Simplex::edge(m, edge3));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge4 = m.edge_tuple_with_vs_and_t(4, 9, 8);
    split(Simplex::edge(m, edge4));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge5 = m.edge_tuple_with_vs_and_t(5, 6, 4);
    split(Simplex::edge(m, edge5));
    REQUIRE(m.is_connectivity_valid());
}

TEST_CASE("split_edge_operation", "[operations][split][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();

    REQUIRE(m.is_connectivity_valid());
    EdgeSplit op(m);

    const Tuple e = m.edge_tuple_with_vs_and_t(0, 1, 1);
    bool split_boundary_edges;
    SECTION("split_boundary_true")
    {
        split_boundary_edges = true;
    }
    SECTION("split_boundary_false")
    {
        split_boundary_edges = false;
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
    }

    const bool success = !op(Simplex::edge(m, e)).empty();
    CHECK(success == split_boundary_edges);
    if (split_boundary_edges) {
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 10);
    } else {
        CHECK(m.get_all(PrimitiveType::Vertex).size() == 9);
    }
}

TEST_CASE("split_return_tuple", "[operations][split][2D]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(1, 2, 0);
        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret));
        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 2);
        CHECK(m.id(ret, PF) == 2);
    }
    SECTION("single_triangle_inverted")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 1, 0);
        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret));
        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(ret, PF) == 2);
    }
    SECTION("three_neighbors")
    {
        DEBUG_TriMesh m = three_neighbors();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 1, 1);
        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret));
        CHECK(m.id(ret, PV) == 6);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 0);
        CHECK(m.id(ret, PF) == 5);
    }
    SECTION("three_neighbors_opposite")
    {
        DEBUG_TriMesh m = three_neighbors();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_with_vs_and_t(2, 1, 3);
        EdgeSplit split(m);
        auto res = split(Simplex::edge(m, edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret));
        CHECK(m.id(ret, PV) == 6);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 5);
        CHECK(m.id(ret, PF) == 5);
    }
}

TEST_CASE("split_multiple_edges", "[operations][split][2D]")
{
    DEBUG_TriMesh mesh;

    SECTION("single_triangle")
    {
        mesh = single_triangle();
    }
    SECTION("quad")
    {
        mesh = quad();
    }
    SECTION("tetrahedron")
    {
        mesh = tetrahedron();
    }
    SECTION("edge_region")
    {
        mesh = edge_region();
    }
    EdgeSplit split(mesh);

    for (size_t i = 0; i < 5; ++i) {
        const std::vector<wmtk::Tuple> edges = mesh.get_all(PE);
        for (const wmtk::Tuple& e : edges) {
            if (!mesh.is_valid(e)) {
                continue;
            }

            split(Simplex::edge(mesh, e));
            REQUIRE(mesh.is_connectivity_valid());
        }
    }
}

TEST_CASE("split_modified_primitives", "[operations][split]")
{
    DEBUG_TriMesh m = edge_region();
    EdgeSplit op(m);

    const Tuple e = m.edge_tuple_with_vs_and_t(4, 5, 2);
    const auto ret = op(Simplex::edge(m, e));
    REQUIRE(!ret.empty());
    CHECK(ret.size() == 1);
    CHECK(ret[0].primitive_type() == PrimitiveType::Vertex);
    CHECK(m.id(ret[0]) == 10);
}


TEST_CASE("split_no_topology_trimesh", "[operations][split]")
{
    const int64_t initial_size = 20;
    DEBUG_TriMesh m = [](int64_t size) {
        TriMesh m;
        m.initialize_free(size);
        return m;
    }(initial_size);
    int64_t size = initial_size;
    for (Tuple edge : m.get_all(PrimitiveType::Triangle)) {
        EdgeSplit op(m);
        REQUIRE(!op(simplex::Simplex(m, PrimitiveType::Edge, edge)).empty());
        size++;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(is_free(m));
        CHECK(m.get_all(PrimitiveType::Triangle).size() == size);
    }
}
