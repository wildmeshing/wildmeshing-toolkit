#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitWithTag.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapBase.hpp>
#include <wmtk/operations/tri_mesh/FaceSplit.hpp>
#include <wmtk/operations/tri_mesh/FaceSplitAtMidPoint.hpp>
#include <wmtk/operations/tri_mesh/FaceSplitWithTag.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::tests;

using TM = TriMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 1);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarData ear1 = face_data.ears[0];
        TMOE::EarData ear2 = face_data.ears[1];
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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 0);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarData ear1 = face_data.ears[0];
        TMOE::EarData ear2 = face_data.ears[1];
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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        const std::vector<TMOE::IncidentFaceData>& face_datas = executor.incident_face_datas();
        REQUIRE(face_datas.size() == 1);
        const TMOE::IncidentFaceData& face_data = face_datas[0];
        CHECK(face_data.opposite_vid == 0);
        CHECK(face_data.fid == 0);
        REQUIRE(face_data.ears.size() == 2);
        TMOE::EarData ear1 = face_data.ears[0];
        TMOE::EarData ear2 = face_data.ears[1];
        CHECK(ear1.fid == 1);
        CHECK(ear1.eid > -1);
        CHECK(ear2.fid == 2);
        CHECK(ear2.eid > -1);
    }
}

TEST_CASE("get_split_simplices_to_delete", "[operations][split][2D]")
{
    SECTION("single_triangle")
    {
        const DEBUG_TriMesh m = single_triangle();
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);

        std::array<std::vector<long>, 3> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 1);

        const long edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m._debug_id(edge, PE));
        const long face_to_delete = ids_to_delete[2][0];
        CHECK(face_to_delete == m._debug_id(edge, PF));
    }
    SECTION("hex_plus_two")
    {
        const DEBUG_TriMesh m = hex_plus_two();
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);

        std::array<std::vector<long>, 3> ids_to_delete =
            TMOE::get_split_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 0);
        REQUIRE(ids_to_delete[1].size() == 1);
        REQUIRE(ids_to_delete[2].size() == 2);

        const long edge_to_delete = ids_to_delete[1][0];
        CHECK(edge_to_delete == m._debug_id(edge, PE));

        // compare expected face ids with the actual ones that should be deleted
        std::set<long> fid_expected;
        fid_expected.insert(m._debug_id(edge, PF));
        fid_expected.insert(m._debug_id(m.switch_face(edge), PF));

        std::set<long> fid_actual;
        for (const long& f : ids_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
}

TEST_CASE("get_collapse_simplices_to_delete", "[operations][collapse][2D]")
{
    SECTION("interior_edge")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);

        std::array<std::vector<long>, 3> ids_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 3);
        REQUIRE(ids_to_delete[2].size() == 2);

        // V
        const long vertex_to_delete = ids_to_delete[0][0];
        CHECK(vertex_to_delete == m._debug_id(edge, PV));

        // E
        std::set<long> eid_expected;
        eid_expected.insert(m._debug_id(edge, PE));
        eid_expected.insert(m._debug_id(m.switch_edge(edge), PE));
        eid_expected.insert(m._debug_id(m.switch_edge(m.switch_face(edge)), PE));

        std::set<long> eid_actual;
        for (const long& e : ids_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        std::set<long> fid_expected;
        fid_expected.insert(m._debug_id(edge, PF));
        fid_expected.insert(m._debug_id(m.switch_face(edge), PF));

        std::set<long> fid_actual;
        for (const long& f : ids_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
    SECTION("boundary_edge")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_between_v1_v2(7, 8, 6);

        std::array<std::vector<long>, 3> ids_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(ids_to_delete[0].size() == 1);
        REQUIRE(ids_to_delete[1].size() == 2);
        REQUIRE(ids_to_delete[2].size() == 1);

        // V
        const long vertex_to_delete = ids_to_delete[0][0];
        CHECK(vertex_to_delete == m._debug_id(edge, PV));

        // E
        std::set<long> eid_expected;
        eid_expected.insert(m._debug_id(edge, PE));
        eid_expected.insert(m._debug_id(m.switch_edge(edge), PE));

        std::set<long> eid_actual;
        for (const long& e : ids_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        const long face_to_delete = ids_to_delete[2][0];
        CHECK(face_to_delete == m._debug_id(edge, PF));
    }
    SECTION("interior_edge_incident_to_boundary")
    {
        const DEBUG_TriMesh m = edge_region();
        Tuple edge = m.edge_tuple_between_v1_v2(7, 4, 5);

        std::array<std::vector<long>, 3> sc_to_delete =
            TMOE::get_collapse_simplices_to_delete(edge, m);

        REQUIRE(sc_to_delete[0].size() == 1);
        REQUIRE(sc_to_delete[1].size() == 3);
        REQUIRE(sc_to_delete[2].size() == 2);

        // V
        const long vertex_to_delete = sc_to_delete[0][0];
        CHECK(vertex_to_delete == m._debug_id(edge, PV));

        // E
        std::set<long> eid_expected;
        eid_expected.insert(m._debug_id(edge, PE));
        eid_expected.insert(m._debug_id(m.switch_edge(edge), PE));
        eid_expected.insert(m._debug_id(m.switch_edge(m.switch_face(edge)), PE));

        std::set<long> eid_actual;
        for (const long& e : sc_to_delete[1]) {
            CHECK(eid_expected.find(e) != eid_expected.end());
            eid_actual.insert(e);
        }
        CHECK(eid_actual.size() == eid_expected.size());

        // F
        std::set<long> fid_expected;
        fid_expected.insert(m._debug_id(edge, PF));
        fid_expected.insert(m._debug_id(m.switch_face(edge), PF));

        std::set<long> fid_actual;
        for (const long& f : sc_to_delete[2]) {
            CHECK(fid_expected.find(f) != fid_expected.end());
            fid_actual.insert(f);
        }
        CHECK(fid_actual.size() == fid_expected.size());
    }
}

TEST_CASE("delete_simplices", "[operations][2D]")
{
    // delete for split

    // things can be marked as deleted but will still have the connectivity information
    DEBUG_TriMesh m = two_neighbors();
    REQUIRE(m.is_connectivity_valid());
    Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
    std::vector<std::vector<long>> simplices_to_delete(3);
    simplices_to_delete[1].emplace_back(m._debug_id(edge, PE));
    simplices_to_delete[2].emplace_back(m._debug_id(edge, PF));

    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    auto executor = m.get_tmoe(edge, hash_accessor);

    // new way of getting simplices
    executor.simplex_ids_to_delete = TMOE::get_split_simplices_to_delete(edge, m);

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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

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
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

        REQUIRE(executor.flag_accessors.size() == 3);
        REQUIRE(executor.incident_vids().size() == 2);
        REQUIRE(executor.incident_vids()[0] == 1);
        REQUIRE(executor.incident_vids()[1] == 2);
        REQUIRE(executor.operating_edge_id() == m._debug_id(edge, PE));
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
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    auto executor = m.get_tmoe(edge, hash_accessor);
    auto ff_accessor_before = m.create_base_accessor<long>(m.f_handle(PF));
    REQUIRE(ff_accessor_before.vector_attribute(1)(2) == 2);
    TMOE::EarData ear{1, m._debug_id(edge, PE)};
    executor.update_ids_in_ear(ear, 3, 2);
    auto ff_accessor_after = m.create_base_accessor<long>(m.f_handle(PF));
    REQUIRE(ff_accessor_after.vector_attribute(1)(2) == 3);
}

TEST_CASE("hash_update", "[operations][2D]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(0, 2, 0);

        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        // auto& ha = executor.hash_accessor;

        CHECK(m.get_cell_hash_slow(0) == 0);

        executor.update_cell_hash();

        CHECK(m.get_cell_hash_slow(0) == 1);
    }
    SECTION("edge_region")
    {
        DEBUG_TriMesh m = edge_region();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(3, 7, 5);

        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        // auto& ha = executor.hash_accessor;

        CHECK(m.get_cell_hash_slow(0) == 0);
        CHECK(m.get_cell_hash_slow(1) == 0);
        CHECK(m.get_cell_hash_slow(2) == 0);
        CHECK(m.get_cell_hash_slow(3) == 0);
        CHECK(m.get_cell_hash_slow(4) == 0);
        CHECK(m.get_cell_hash_slow(5) == 0);
        CHECK(m.get_cell_hash_slow(6) == 0);
        CHECK(m.get_cell_hash_slow(7) == 0);
        CHECK(m.get_cell_hash_slow(8) == 0);
        CHECK(m.get_cell_hash_slow(9) == 0);

        executor.update_cell_hash();

        CHECK(m.get_cell_hash_slow(0) == 1);
        CHECK(m.get_cell_hash_slow(1) == 1);
        CHECK(m.get_cell_hash_slow(2) == 1);
        CHECK(m.get_cell_hash_slow(3) == 0);
        CHECK(m.get_cell_hash_slow(4) == 0);
        CHECK(m.get_cell_hash_slow(5) == 1);
        CHECK(m.get_cell_hash_slow(6) == 1);
        CHECK(m.get_cell_hash_slow(7) == 1);
        CHECK(m.get_cell_hash_slow(8) == 0);
        CHECK(m.get_cell_hash_slow(9) == 0);
    }
}

//////////// SPLIT TESTS ////////////
TEST_CASE("connect_faces_across_spine", "[operations][split][2D]")
{
    DEBUG_TriMesh m = interior_edge();
    m.reserve_attributes(PF, 10);
    REQUIRE(m.is_connectivity_valid());
    const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    auto executor = m.get_tmoe(edge, hash_accessor);
    auto& incident_face_datas = executor.m_incident_face_datas;

    REQUIRE(executor.incident_face_datas().size() == 2);

    const auto new_fids = executor.request_simplex_indices(PF, 4);
    long& f0_top = incident_face_datas[0].split_f[0];
    long& f1_top = incident_face_datas[0].split_f[1];
    long& f0_bottom = incident_face_datas[1].split_f[0];
    long& f1_bottom = incident_face_datas[1].split_f[1];
    f0_top = new_fids[0];
    f1_top = new_fids[2];
    f0_bottom = new_fids[1];
    f1_bottom = new_fids[3];

    executor.connect_faces_across_spine();

    const long local_eid_top = 0;
    const long local_eid_bottom = 1;

    auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));

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
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        auto& incident_face_datas = executor.m_incident_face_datas;

        //  create new vertex
        std::vector<long> new_vids = executor.request_simplex_indices(PV, 1);
        REQUIRE(new_vids.size() == 1);
        executor.split_new_vid = new_vids[0];

        // create new edges
        std::vector<long> spine_eids = executor.request_simplex_indices(PE, 2);
        REQUIRE(spine_eids.size() == 2);
        std::copy(spine_eids.begin(), spine_eids.end(), executor.split_spine_eids.begin());

        std::vector<std::array<long, 2>> new_fids;
        REQUIRE(incident_face_datas.size() == 1);
        for (size_t i = 0; i < incident_face_datas.size(); ++i) {
            auto& face_data = incident_face_datas[i];
            executor.replace_incident_face(face_data);
        }
        REQUIRE(incident_face_datas.size() == 1);

        const long& f0 = incident_face_datas[0].split_f[0];
        const long& f1 = incident_face_datas[0].split_f[1];
        const long& se0 = spine_eids[0];
        const long& se1 = spine_eids[1];
        const long& ee0 = incident_face_datas[0].ears[0].eid;
        const long& ee1 = incident_face_datas[0].ears[1].eid;

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));
        const auto fv0 = fv_accessor.vector_attribute(f0);
        const auto fv1 = fv_accessor.vector_attribute(f1);
        CHECK(fv0[0] == 0);
        CHECK(fv0[1] == 1);
        CHECK(fv0[2] == executor.split_new_vid);

        CHECK(fv1[0] == 0);
        CHECK(fv1[1] == executor.split_new_vid);
        CHECK(fv1[2] == 2);

        // the new fids generated are in top-down left-right order
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));
        const auto ff0 = ff_accessor.vector_attribute(f0);
        const auto ff1 = ff_accessor.vector_attribute(f1);
        CHECK(ff0[0] == -1);
        CHECK(ff0[1] == f1);
        CHECK(ff0[2] == -1);

        CHECK(ff1[0] == -1);
        CHECK(ff1[1] == -1);
        CHECK(ff1[2] == f0);

        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PE));
        const auto fe0 = fe_accessor.vector_attribute(f0);
        const auto fe1 = fe_accessor.vector_attribute(f1);

        CHECK(fe0[0] == se0);
        CHECK(fe0[1] == 5);
        CHECK(fe0[2] == ee0);

        CHECK(fe1[0] == se1);
        CHECK(fe1[1] == ee1);
        CHECK(fe1[2] == 5);

        auto vf_accessor = m.create_base_accessor<long>(m.vf_handle());
        CHECK(vf_accessor.scalar_attribute(executor.split_new_vid) == f0);
        CHECK(vf_accessor.scalar_attribute(0) == f0);
        CHECK(vf_accessor.scalar_attribute(1) == f0);
        CHECK(vf_accessor.scalar_attribute(2) == f1);

        auto ef_accessor = m.create_base_accessor<long>(m.ef_handle());
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
        Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        auto& incident_face_datas = executor.m_incident_face_datas;

        // create new vertex
        std::vector<long> new_vids = executor.request_simplex_indices(PV, 1);
        REQUIRE(new_vids.size() == 1);
        const long v_new = new_vids[0];
        executor.split_new_vid = new_vids[0];

        // create new edges
        std::vector<long> spine_eids = executor.request_simplex_indices(PE, 2);
        REQUIRE(spine_eids.size() == 2);
        std::copy(spine_eids.begin(), spine_eids.end(), executor.split_spine_eids.begin());

        for (size_t i = 0; i < incident_face_datas.size(); ++i) {
            auto& face_data = incident_face_datas[i];
            executor.replace_incident_face(face_data);
        }
        REQUIRE(incident_face_datas.size() == 2);

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));
        auto fe_accessor = m.create_base_accessor<long>(m.f_handle(PE));
        auto ff_accessor = m.create_base_accessor<long>(m.f_handle(PF));

        auto vf_accessor = m.create_base_accessor<long>(m.vf_handle());

        auto ef_accessor = m.create_base_accessor<long>(m.ef_handle());

        const long& se0 = spine_eids[0];
        const long& se1 = spine_eids[1];

        // top
        {
            const long& f0 = incident_face_datas[0].split_f[0];
            const long& f1 = incident_face_datas[0].split_f[1];
            const long& ee0 = incident_face_datas[0].ears[0].eid;
            const long& ee1 = incident_face_datas[0].ears[1].eid;

            const auto fv0 = fv_accessor.vector_attribute(f0);
            const auto fv1 = fv_accessor.vector_attribute(f1);
            CHECK(fv0[0] == 0);
            CHECK(fv0[1] == 1);
            CHECK(fv0[2] == v_new);

            CHECK(fv1[0] == 0);
            CHECK(fv1[1] == v_new);
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
            CHECK(fe0[0] == spine_eids[0]);
            CHECK(fe0[1] == 9);
            CHECK(fe0[2] == ee0);

            CHECK(fe1[0] == spine_eids[1]);
            CHECK(fe1[1] == ee1);
            CHECK(fe1[2] == 9);

            CHECK(vf_accessor.scalar_attribute(0) == f0);

            CHECK(ef_accessor.scalar_attribute(ee0) == f0);
            CHECK(ef_accessor.scalar_attribute(ee1) == f1);
            CHECK(ef_accessor.scalar_attribute(9) == f0);
        }
        // bottom
        {
            const long& f0 = incident_face_datas[1].split_f[0];
            const long& f1 = incident_face_datas[1].split_f[1];
            const long& ee0 = incident_face_datas[1].ears[0].eid;
            const long& ee1 = incident_face_datas[1].ears[1].eid;

            const auto fv0 = fv_accessor.vector_attribute(f0);
            const auto fv1 = fv_accessor.vector_attribute(f1);

            CHECK(fv0[0] == 1);
            CHECK(fv0[1] == 4);
            CHECK(fv0[2] == v_new);

            CHECK(fv1[0] == v_new);
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

            CHECK(vf_accessor.scalar_attribute(v_new) == f0);
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
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        const long edge_id = m._debug_id(edge, PE);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

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
            tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
            tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
            tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
            m.initialize(tris);
        }
        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        const long edge_id = m._debug_id(edge, PE);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);

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
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();

    REQUIRE(m.is_connectivity_valid());

    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    m.split_edge(edge, hash_accessor);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge2 = m.edge_tuple_between_v1_v2(3, 0, 0);
    m.split_edge(edge2, hash_accessor);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge3 = m.edge_tuple_between_v1_v2(4, 7, 6);
    REQUIRE(m.is_valid_slow(edge3));
    m.split_edge(edge3, hash_accessor);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge4 = m.edge_tuple_between_v1_v2(4, 9, 8);
    m.split_edge(edge4, hash_accessor);
    REQUIRE(m.is_connectivity_valid());

    Tuple edge5 = m.edge_tuple_between_v1_v2(5, 6, 4);
    m.split_edge(edge5, hash_accessor);
    REQUIRE(m.is_connectivity_valid());
}

TEST_CASE("split_edge_operation", "[operations][split][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();

    REQUIRE(m.is_connectivity_valid());
    OperationSettings<tri_mesh::EdgeSplit> op_settings(m);

    const Tuple e = m.edge_tuple_between_v1_v2(0, 1, 1);
    SECTION("split_boundary_true")
    {
        op_settings.split_boundary_edges = true;
    }
    SECTION("split_boundary_false")
    {
        op_settings.split_boundary_edges = false;
    }
    op_settings.create_invariants();

    tri_mesh::EdgeSplit op(m, Simplex::edge(e), op_settings);
    const bool success = op();
    CHECK(success == op_settings.split_boundary_edges);
    if (op_settings.split_boundary_edges) {
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

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        const Tuple ret = m.split_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret, hash_accessor));
        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 2);
        CHECK(m.id(ret, PF) == 2);
    }
    SECTION("single_triangle_inverted")
    {
        DEBUG_TriMesh m = single_triangle();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        const Tuple ret = m.split_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret, hash_accessor));
        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(ret, PF) == 2);
    }
    SECTION("three_neighbors")
    {
        DEBUG_TriMesh m = three_neighbors();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 1);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        const Tuple ret = m.split_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret, hash_accessor));
        CHECK(m.id(ret, PV) == 6);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 0);
        CHECK(m.id(ret, PF) == 5);
    }
    SECTION("three_neighbors_opposite")
    {
        DEBUG_TriMesh m = three_neighbors();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 3);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        const Tuple ret = m.split_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());
        REQUIRE(m.is_valid(ret, hash_accessor));
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

    for (size_t i = 0; i < 10; ++i) {
        const std::vector<wmtk::Tuple> edges = mesh.get_all(PE);
        for (const wmtk::Tuple& e : edges) {
            if (!mesh.is_valid_slow(e)) {
                continue;
            }

            Accessor<long> hash_accessor = mesh.get_cell_hash_accessor();
            mesh.split_edge(e, hash_accessor);
            REQUIRE(mesh.is_connectivity_valid());
        }
    }
}

//////////// COLLAPSE TESTS ////////////

TEST_CASE("collapse_edge", "[operations][collapse][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();
    REQUIRE(m.is_connectivity_valid());

    SECTION("interior_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
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
    }
    SECTION("edge_to_boundary")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 0, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 4));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(0)) == 0);
        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(2)[0] == 0);
        CHECK(fv_accessor.vector_attribute(5)[2] == 0);
        CHECK(fv_accessor.vector_attribute(6)[2] == 0);
        CHECK(fv_accessor.vector_attribute(7)[0] == 0);
    }
    SECTION("edge_from_boundary_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        OperationSettings<tri_mesh::EdgeCollapse> op_settings(m);
        op_settings.create_invariants();
        tri_mesh::EdgeCollapse op(m, Simplex::edge(edge), op_settings);
        const bool success = op();
        CHECK(success);
    }
    SECTION("edge_from_boundary_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        OperationSettings<tri_mesh::EdgeCollapse> op_settings(m);
        op_settings.collapse_boundary_vertex_to_interior = false;
        op_settings.create_invariants();

        tri_mesh::EdgeCollapse op(m, Simplex::edge(edge), op_settings);
        const bool success = op();
        CHECK(!success);
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        m.collapse_edge(edge, hash_accessor);
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 0));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(0)[2] == 1);
    }
    SECTION("boundary_edge_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        OperationSettings<tri_mesh::EdgeCollapse> op_settings(m);
        op_settings.create_invariants();
        tri_mesh::EdgeCollapse op(m, Simplex::edge(edge), op_settings);
        const bool success = op();
        CHECK(success);
    }
    SECTION("boundary_edge_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        OperationSettings<tri_mesh::EdgeCollapse> op_settings(m);
        op_settings.collapse_boundary_edges = false;
        op_settings.create_invariants();

        tri_mesh::EdgeCollapse op(m, Simplex::edge(edge), op_settings);
        const bool success = op();
        CHECK(!success);
    }
}

TEST_CASE("collapse_return_tuple", "[operations][collapse][2D]")
{
    DEBUG_TriMesh m = edge_region();
    Accessor<long> hash_accessor = m.get_cell_hash_accessor();
    SECTION("interior")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
        const Tuple ret = m.collapse_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_valid_slow(ret));
        REQUIRE(m.is_connectivity_valid());
        // CHECK(op.is_return_tuple_from_left_ear() == false);

        CHECK(m.id(ret, PV) == 5);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(ret, PF) == 1);
    }
    SECTION("from_boundary")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(3, 4, 0);
        const Tuple ret = m.collapse_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());
        // CHECK(op.is_return_tuple_from_left_ear() == false);

        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);
        CHECK(m.id(ret, PF) == 1);
    }
    SECTION("to_boundary")
    {
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(4, 3, 0);
        const Tuple ret = m.collapse_edge(edge, hash_accessor).m_output_tuple;
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);
        CHECK(m.id(ret, PF) == 1);
    }
}

TEST_CASE("swap_edge", "[operations][swap][2D]")
{
    using namespace operations;
    using namespace tri_mesh;

    SECTION("counter_clockwise")
    {
        DEBUG_TriMesh m = interior_edge();
        OperationSettings<EdgeSwapBase> settings(m);
        settings.create_invariants();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        EdgeSwapBase op(m, Simplex::edge(edge), settings);
        CHECK(op.name() == "tri_mesh_edge_swap_base");
        REQUIRE(op());
        const Tuple ret = op.return_tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);

        auto fv_accessor = m.create_const_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        auto f5_fv = fv_accessor.vector_attribute(5);
        CHECK(f5_fv[0] == 1);
        CHECK(f5_fv[1] == 4);
        CHECK(f5_fv[2] == 0);
        auto f6_fv = fv_accessor.vector_attribute(6);
        CHECK(f6_fv[0] == 0);
        CHECK(f6_fv[1] == 4);
        CHECK(f6_fv[2] == 2);
    }
    SECTION("clockwise")
    {
        DEBUG_TriMesh m = interior_edge();
        OperationSettings<EdgeSwapBase> settings(m);
        settings.create_invariants();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 2);
        EdgeSwapBase op(m, Simplex::edge(edge), settings);
        REQUIRE(op());
        const Tuple ret = op.return_tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 0);
        CHECK(m.id(m.switch_vertex(ret), PV) == 4);

        auto fv_accessor = m.create_const_base_accessor<long>(m.f_handle(PrimitiveType::Vertex));
        auto f5_fv = fv_accessor.vector_attribute(5);
        auto f6_fv = fv_accessor.vector_attribute(6);
        CHECK(f5_fv[0] == 0);
        CHECK(f5_fv[1] == 1);
        CHECK(f5_fv[2] == 4);
        CHECK(f6_fv[0] == 0);
        CHECK(f6_fv[1] == 4);
        CHECK(f6_fv[2] == 2);
    }
    SECTION("single_triangle_fail")
    {
        DEBUG_TriMesh m = single_triangle();
        OperationSettings<EdgeSwapBase> settings(m);
        settings.create_invariants();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        EdgeSwapBase op(m, Simplex::edge(edge), settings);
        REQUIRE_FALSE(op());
        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("tetrahedron_fail")
    {
        DEBUG_TriMesh m = tetrahedron();
        OperationSettings<EdgeSwapBase> settings(m);
        settings.create_invariants();
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 1);
        EdgeSwapBase op(m, Simplex::edge(edge), settings);
        REQUIRE_FALSE(op());
        REQUIRE(m.is_connectivity_valid());
    }
}

TEST_CASE("split_face", "[operations][split][2D]")
{
    using namespace operations;
    SECTION("split_single_triangle")
    {
        //         0
        //        / \ .
        //       2   1
        //      /  0  \ .
        //     /       \ .
        //  1  ----0---- 2
        //
        // this case covered the on boundary case
        DEBUG_TriMesh m = single_triangle();
        Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        // spdlog::info("{}", m.id(f, PV));
        // spdlog::info("{}", m.id(m.switch_vertex(f), PV));
        // spdlog::info("{}", m.id(m.switch_vertex(m.switch_edge(f)), PV));
        OperationSettings<tri_mesh::FaceSplit> settings(m);
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplit op(m, Simplex::face(f), settings);
        CHECK(op.name().compare("tri_mesh_face_split") == 0);
        bool is_success = op();
        Tuple ret = op.return_tuple();
        // spdlog::info("{}", m.id(ret, PV));
        // spdlog::info("{}", m.id(m.switch_vertex(ret), PV));
        // spdlog::info("{}", m.id(m.switch_vertex(m.switch_edge(ret)), PV));
        CHECK(is_success);
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(SimplicialComplex::vertex_one_ring(m, ret).size() == 3);
    }
    SECTION("split_face_in_quad")
    {
        //  3--1--- 0
        //   |     / \ .
        //   2 f1 /2   1
        //   |  0/ f0  \ .
        //   |  /       \ .
        //  1  ----0---- 2
        //
        DEBUG_TriMesh m = quad();
        Tuple f = m.edge_tuple_between_v1_v2(1, 0, 1);
        OperationSettings<tri_mesh::FaceSplit> settings(m);
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplit op(m, Simplex::face(f), settings);
        bool is_success = op();
        CHECK(is_success);
        CHECK(m.get_all(PV).size() == 5);
        CHECK(m.id(op.return_tuple(), PV) == 5);
        CHECK(m.id(m.switch_vertex(op.return_tuple()), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(op.return_tuple())), PV) == 0);

        /* TODO: figure out how to make the new invariant / modified_primitives fits with this
        Simplex v(PrimitiveType::Vertex, op.return_tuple());
        auto sc = SimplicialComplex::open_star(m, v);
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Face);
            for (const Tuple& t : modified_tuples) {
                bool t_exist = false;
                int times = 0;
                for (const Simplex& s : sc.get_simplices(PrimitiveType::Face)) {
                    if (m.id(t, PrimitiveType::Face) == m.id(s.tuple(), PrimitiveType::Face)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Edge);
            for (const Tuple& t : modified_tuples) {
                bool t_exist = false;
                int times = 0;
                for (const Simplex& s : sc.get_simplices(PrimitiveType::Edge)) {
                    if (m.id(t, PrimitiveType::Edge) == m.id(s.tuple(), PrimitiveType::Edge)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        {
            std::vector<Tuple> modified_tuples = op.modified_primitives(PrimitiveType::Vertex);
            for (const Tuple& t : modified_tuples) {
                bool t_exist = false;
                int times = 0;
                for (const Simplex& s : sc.get_simplices(PrimitiveType::Vertex)) {
                    if (m.id(t, PrimitiveType::Vertex) == m.id(s.tuple(), PrimitiveType::Vertex)) {
                        t_exist = true;
                        break;
                    }
                }
                CHECK(t_exist);
            }
        }
        */
    }
    SECTION("split_in_diamond_with_attribute")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region_with_position();

        Tuple f0 = m.edge_tuple_between_v1_v2(3, 4, 0); // on boundary
        Tuple f1 = m.edge_tuple_between_v1_v2(8, 9, 8); // out boundary
        Tuple f2 = m.edge_tuple_between_v1_v2(4, 8, 7); // overlap of f0 and f1

        MeshAttributeHandle<long> attri_handle =
            m.register_attribute<long>("test_attribute", PF, 1);
        Accessor<long> acc_attri = m.create_accessor<long>(attri_handle);
        acc_attri.scalar_attribute(f1) = 1;

        OperationSettings<tri_mesh::FaceSplitAtMidPoint> settings(m);
        settings.position = m.get_attribute_handle<double>("vertices", PV);
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplitAtMidPoint op0(m, Simplex::face(f0), settings);
        wmtk::operations::tri_mesh::FaceSplitAtMidPoint op1(m, Simplex::face(f1), settings);
        wmtk::operations::tri_mesh::FaceSplitAtMidPoint op2(m, Simplex::face(f2), settings);
        CHECK(op0.name().compare("tri_mesh_split_face_at_midpoint") == 0);
        CHECK(op1.name().compare("tri_mesh_split_face_at_midpoint") == 0);
        CHECK(op2.name().compare("tri_mesh_split_face_at_midpoint") == 0);
        CHECK(op0());
        CHECK(op1());
        CHECK(!op2());

        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_attri.scalar_attribute(t) == 0);
        }

        if (false) {
            const std::filesystem::path data_dir = WMTK_DATA_DIR;
            ParaviewWriter writer(
                data_dir / "split_in_diamond_with_attribute_result",
                "vertices",
                m,
                true,
                true,
                true,
                false);
            m.serialize(writer);
        }
    }
    SECTION("split_single_triangle_at_mid_point")
    {
        //         0
        //        / \ .
        //       2   1
        //      /  0  \ .
        //     /       \ .
        //  1  ----0---- 2
        //
        // this case covered the on boundary case
        // V.row(0) << 0, 0, 0;
        // V.row(1) << 1, 0, 0;
        // V.row(2) << 0.5, 0.866, 0;
        DEBUG_TriMesh m = single_equilateral_triangle(3);
        Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        OperationSettings<tri_mesh::FaceSplitAtMidPoint> settings(m);
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);
        settings.position = pos_handle;
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplitAtMidPoint op(m, Simplex::face(f), settings);
        bool is_success = op();
        Tuple ret = op.return_tuple();
        CHECK(is_success);
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(SimplicialComplex::vertex_one_ring(m, ret).size() == 3);
        Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);
        CHECK(acc_pos.vector_attribute(ret).x() == 0.5);
        CHECK(acc_pos.vector_attribute(m.switch_vertex(ret)).x() == 1);
    }
    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_on")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region_with_position();
        Tuple f = m.edge_tuple_between_v1_v2(3, 4, 0);
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);
        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>("edge_tag", PE, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PV, 1);
        Accessor<long> acc_todo = m.create_accessor<long>(todo_handle);
        acc_todo.scalar_attribute(f) = 1;
        OperationSettings<tri_mesh::FaceSplitWithTag> settings(m);
        settings.edge_tag = edge_tag_handle;
        settings.vertex_tag = vertex_tag_handle;
        settings.embedding_tag_value = 0;
        settings.face_split_settings.position = pos_handle;
        settings.need_embedding_tag_value = true;
        settings.split_todo = todo_handle;
        settings.split_vertex_tag_value = 3;
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplitWithTag op(m, Simplex::face(f), settings);
        CHECK(op.name().compare("tri_mesh_split_face_with_tag") == 0);
        CHECK(op());
        CHECK(m.get_all(PF).size() == 12);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }
    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_off")
    {
        //         0
        //        / \ .
        //       2   1
        //      /  0  \ .
        //     /       \ .
        //  1  ----0---- 2
        //
        // this case covered the on boundary case
        // V.row(0) << 0, 0, 0;
        // V.row(1) << 1, 0, 0;
        // V.row(2) << 0.5, 0.866, 0;
        DEBUG_TriMesh m = single_equilateral_triangle(3);
        Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);
        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>("edge_tag", PE, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PV, 1);
        Accessor<long> acc_todo = m.create_accessor<long>(todo_handle);
        Accessor<long> acc_edge_tag = m.create_accessor<long>(edge_tag_handle);
        Accessor<long> acc_vertex_tag = m.create_accessor<long>(vertex_tag_handle);
        acc_todo.scalar_attribute(f) = 1;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 1;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 2;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;
        OperationSettings<tri_mesh::FaceSplitWithTag> settings(m);
        settings.edge_tag = edge_tag_handle;
        settings.vertex_tag = vertex_tag_handle;
        settings.embedding_tag_value = 0;
        settings.face_split_settings.position = pos_handle;
        settings.need_embedding_tag_value = false;
        settings.split_todo = todo_handle;
        settings.split_vertex_tag_value = 3;
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplitWithTag op(m, Simplex::face(f), settings);
        CHECK(op());
        CHECK(acc_edge_tag.scalar_attribute(op.return_tuple()) == 1);
        CHECK(acc_edge_tag.scalar_attribute(m.switch_edge(op.return_tuple())) == 3);
        CHECK(acc_edge_tag.scalar_attribute(m.switch_edge(m.switch_face(op.return_tuple()))) == 2);
        CHECK(m.get_all(PF).size() == 3);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }
    SECTION("should fail with todo tag 0")
    {
        DEBUG_TriMesh m = single_equilateral_triangle(3);
        Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);
        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>("edge_tag", PE, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PV, 1);
        Accessor<long> acc_todo = m.create_accessor<long>(todo_handle);
        Accessor<long> acc_vertex_tag = m.create_accessor<long>(vertex_tag_handle);
        acc_todo.scalar_attribute(f) = 0;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 1;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 2;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;
        OperationSettings<tri_mesh::FaceSplitWithTag> settings(m);
        settings.edge_tag = edge_tag_handle;
        settings.vertex_tag = vertex_tag_handle;
        settings.embedding_tag_value = 0;
        settings.face_split_settings.position = pos_handle;
        settings.need_embedding_tag_value = false;
        settings.split_todo = todo_handle;
        settings.split_vertex_tag_value = 3;
        settings.create_invariants();
        wmtk::operations::tri_mesh::FaceSplitWithTag op(m, Simplex::face(f), settings);
        CHECK(!op());
    }
}

TEST_CASE("split_edge_operation_with_tag", "[operations][split][2D]")
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
    using namespace operations;

    Eigen::MatrixXd V(5, 3);
    V << 0, 0, 0, -1, -1, 0, 1, -1, 0, -1, 1, 0, 1, -1, 0;
    DEBUG_TriMesh m = interior_edge();
    OperationSettings<tri_mesh::EdgeSplitWithTag> settings(m);
    wmtk::MeshAttributeHandle<long> edge_handle =
        m.register_attribute<long>(std::string("edge_tag"), PE, 1);
    wmtk::MeshAttributeHandle<long> vertex_handle =
        m.register_attribute<long>(std::string("vertex_tag"), PV, 1);
    wmtk::MeshAttributeHandle<long> todo_handle =
        m.register_attribute<long>(std::string("todo_tag"), PE, 1);
    wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    settings.edge_tag = edge_handle;
    settings.vertex_tag = vertex_handle;
    settings.embedding_tag_value = -1;
    settings.need_embedding_tag_value = true;
    settings.split_at_midpoint_settings.position =
        m.get_attribute_handle<double>(std::string("vertices"), PV);
    settings.split_at_midpoint_settings.split_boundary_edges = true;
    settings.split_edge_tag_value = -2;
    settings.split_vertex_tag_value = -3;
    settings.split_todo = todo_handle;
    settings.create_invariants();
    SECTION("should all fail")
    {
        for (Tuple t : m.get_all(PV)) {
            wmtk::operations::tri_mesh::EdgeSplitWithTag op(m, Simplex::edge(t), settings);
            CHECK(op.name().compare("tri_mesh_edge_split_with_tag") == 0);
            CHECK(!op());
        }
    }

    SECTION("check the embedding value and the operations should only success twice -- need "
            "embedding = true")
    {
        std::vector<Tuple> edges = m.get_all(PE);
        wmtk::Accessor<long> acc = m.create_accessor(todo_handle);
        wmtk::Accessor<long> acc_e = m.create_accessor(edge_handle);
        wmtk::Accessor<long> acc_v = m.create_accessor(vertex_handle);
        Tuple e0 = m.edge_tuple_between_v1_v2(1, 0, 1);
        Tuple e1 = m.edge_tuple_between_v1_v2(1, 2, 0);
        acc.scalar_attribute(e0) = 1;
        acc.scalar_attribute(e1) = 1;
        int success_num = 0;
        for (int i = 0; i < 5; i++) {
            // i should be 2, but I set 5 for make sure there are no additional error todo tag
            // created in this process.
            for (const Tuple& t : m.get_all(PE)) {
                wmtk::operations::tri_mesh::EdgeSplitWithTag op(m, Simplex::edge(t), settings);
                if (op()) {
                    // todo marks should be removed
                    CHECK(acc.scalar_attribute(op.return_tuple()) == 0);
                    CHECK(acc.scalar_attribute(m.switch_edge(op.return_tuple())) == 0);
                    CHECK(
                        acc.scalar_attribute(
                            m.switch_edge(m.switch_face(m.switch_edge(op.return_tuple())))) == 0);
                    CHECK(
                        acc.scalar_attribute(m.switch_edge(m.switch_face(op.return_tuple()))) == 0);
                    // new tag value should be marked
                    CHECK(acc_e.scalar_attribute(op.return_tuple()) == -1);
                    CHECK(acc_e.scalar_attribute(m.switch_edge(op.return_tuple())) == -2);
                    CHECK(
                        acc_e.scalar_attribute(
                            m.switch_edge(m.switch_face(m.switch_edge(op.return_tuple())))) == -1);
                    CHECK(
                        acc_e.scalar_attribute(m.switch_edge(m.switch_face(op.return_tuple()))) ==
                        -2);
                    CHECK(acc_v.scalar_attribute(op.return_tuple()) == -3);
                    success_num++;
                }
            }
        }
        CHECK(success_num == 2);
    }
    SECTION("check the embedding value and the operations should only success twice -- need "
            "embedding = false")
    {
        settings.need_embedding_tag_value = false;
        std::vector<Tuple> edges = m.get_all(PE);
        wmtk::Accessor<long> acc = m.create_accessor(todo_handle);
        wmtk::Accessor<long> acc_e = m.create_accessor(edge_handle);
        wmtk::Accessor<long> acc_v = m.create_accessor(vertex_handle);
        Tuple e0 = m.edge_tuple_between_v1_v2(1, 0, 1);
        Tuple e1 = m.edge_tuple_between_v1_v2(1, 2, 0);
        acc.scalar_attribute(e0) = 1;
        acc.scalar_attribute(e1) = 1;
        int success_num = 0;
        for (int i = 0; i < 5; i++) {
            // i should be 2, but I set 5 for make sure there are no additional error todo tag
            // created in this process.
            for (const Tuple& t : m.get_all(PE)) {
                wmtk::operations::tri_mesh::EdgeSplitWithTag op(m, Simplex::edge(t), settings);
                if (op()) {
                    // todo marks should be removed
                    CHECK(acc.scalar_attribute(op.return_tuple()) == 0);
                    CHECK(acc.scalar_attribute(m.switch_edge(op.return_tuple())) == 0);
                    CHECK(
                        acc.scalar_attribute(
                            m.switch_edge(m.switch_face(m.switch_edge(op.return_tuple())))) == 0);
                    CHECK(
                        acc.scalar_attribute(m.switch_edge(m.switch_face(op.return_tuple()))) == 0);
                    // new tag value should be marked
                    CHECK(
                        acc_e.scalar_attribute(op.return_tuple()) ==
                        acc_v.scalar_attribute(m.switch_vertex(op.return_tuple())));
                    CHECK(acc_e.scalar_attribute(m.switch_edge(op.return_tuple())) == -2);
                    CHECK(
                        acc_e.scalar_attribute(
                            m.switch_edge(m.switch_face(m.switch_edge(op.return_tuple())))) ==
                        acc_v.scalar_attribute(m.switch_vertex(
                            m.switch_edge(m.switch_face(m.switch_edge(op.return_tuple()))))));
                    CHECK(
                        acc_e.scalar_attribute(m.switch_edge(m.switch_face(op.return_tuple()))) ==
                        -2);
                    CHECK(acc_v.scalar_attribute(op.return_tuple()) == -3);
                    success_num++;
                }
            }
        }
        CHECK(success_num == 2);
    }
}
