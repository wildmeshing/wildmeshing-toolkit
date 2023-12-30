#include <catch2/catch_test_macros.hpp>
#include <filesystem>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/PredicateAwareSplitNewAttributeStrategy.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/redirect_logger_to_cout.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace operations;

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

    REQUIRE(m.is_connectivity_valid());
    EdgeSplit split(m);

    Tuple edge = m.edge_tuple_between_v1_v2(4, 5, 2);
    split(Simplex::edge(edge));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge2 = m.edge_tuple_between_v1_v2(3, 0, 0);
    split(Simplex::edge(edge));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge3 = m.edge_tuple_between_v1_v2(4, 7, 6);
    REQUIRE(m.is_valid_slow(edge3));
    split(Simplex::edge(edge));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge4 = m.edge_tuple_between_v1_v2(4, 9, 8);
    split(Simplex::edge(edge));
    REQUIRE(m.is_connectivity_valid());

    Tuple edge5 = m.edge_tuple_between_v1_v2(5, 6, 4);
    split(Simplex::edge(edge));
    REQUIRE(m.is_connectivity_valid());
}

TEST_CASE("split_edge_operation", "[operations][split][2D]")
{
    using namespace operations;

    DEBUG_TriMesh m = hex_plus_two();

    REQUIRE(m.is_connectivity_valid());
    EdgeSplit op(m);

    const Tuple e = m.edge_tuple_between_v1_v2(0, 1, 1);
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

    const bool success = !op(Simplex::edge(e)).empty();
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

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        EdgeSplit split(m);
        auto res = split(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        EdgeSplit split(m);
        auto res = split(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        EdgeSplit split(m);
        auto res = split(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        EdgeSplit split(m);
        auto res = split(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
    EdgeSplit split(mesh);

    for (size_t i = 0; i < 10; ++i) {
        const std::vector<wmtk::Tuple> edges = mesh.get_all(PE);
        for (const wmtk::Tuple& e : edges) {
            if (!mesh.is_valid_slow(e)) {
                continue;
            }

            split(Simplex::edge(e));
            REQUIRE(mesh.is_connectivity_valid());
        }
    }
}

TEST_CASE("split_modified_primitives", "[operations][split]")
{
    DEBUG_TriMesh m = edge_region();
    EdgeSplit op(m);

    const Tuple e = m.edge_tuple_between_v1_v2(4, 5, 2);
    // TODOfixme: commented because methods are protected
    // const std::vector<Simplex> unmod = op.unmodified_primitives(Simplex::edge(e));
    // CHECK(unmod.size() == 1);
    REQUIRE(!op(Simplex::edge(e)).empty());
    // const std::vector<Simplex> mod = op.modified_primitives();
    // CHECK(mod.size() == 1);

    // TODOfixme: commented because methods are protected
    // CHECK(unmod[0].primitive_type() == PrimitiveType::Edge);
    // CHECK_FALSE(m.is_valid_slow(unmod[0].tuple()));
    // CHECK(mod[0].primitive_type() == PrimitiveType::Vertex);
    // CHECK(m.is_valid_slow(mod[0].tuple()));
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
    }
    SECTION("edge_to_boundary")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(4, 0, 0);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        EdgeCollapse collapse(m);
        collapse(Simplex::edge(edge));
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

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
    }
    SECTION("edge_from_boundary_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 4, 0);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorVertexInvariant>(m));
        const bool fail = op(Simplex::edge(edge)).empty();
        CHECK(fail);
    }
    SECTION("boundary_edge")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);
        Accessor<long> hash_accessor = m.get_cell_hash_accessor();
        auto executor = m.get_tmoe(edge, hash_accessor);
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op(Simplex::edge(edge));
        REQUIRE(m.is_connectivity_valid());

        auto fv_accessor = m.create_base_accessor<long>(m.f_handle(PV));

        // CHECK_THROWS(m.tuple_from_id(PrimitiveType::Vertex, 0));

        REQUIRE(executor.flag_accessors[2].scalar_attribute(m.tuple_from_face_id(1)) == 0);

        CHECK(fv_accessor.vector_attribute(0)[2] == 1);
    }
    SECTION("boundary_edge_allowed")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        const bool success = !op(Simplex::edge(edge)).empty();
        CHECK(success);
    }
    SECTION("boundary_edge_prohibited")
    {
        const Tuple edge = m.edge_tuple_between_v1_v2(0, 1, 1);

        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        const bool fail = op(Simplex::edge(edge)).empty();
        CHECK(fail);
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
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();

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
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        EdgeCollapse op(m);
        op.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
        REQUIRE(m.is_connectivity_valid());

        CHECK(m.id(ret, PV) == 3);
        CHECK(m.id(m.switch_vertex(ret), PV) == 0);
        CHECK(m.id(ret, PF) == 1);
    }
}

TEST_CASE("swap_edge", "[operations][swap][2D]")
{
    using namespace operations::composite;

    SECTION("counter_clockwise")
    {
        DEBUG_TriMesh m = interior_edge();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));


        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 2);
        auto res = op(Simplex::edge(edge));
        REQUIRE(!res.empty());
        const Tuple ret = res.front().tuple();
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
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        REQUIRE(m.is_connectivity_valid());
        const Tuple edge = m.edge_tuple_between_v1_v2(1, 2, 0);
        REQUIRE(op(Simplex::edge(edge)).empty());
        REQUIRE(m.is_connectivity_valid());
    }
    SECTION("tetrahedron_fail")
    {
        DEBUG_TriMesh m = tetrahedron();
        TriEdgeSwap op(m);
        op.add_invariant(std::make_shared<InteriorEdgeInvariant>(m));
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        REQUIRE(m.is_connectivity_valid());

        const Tuple edge = m.edge_tuple_between_v1_v2(2, 1, 1);
        REQUIRE(op(Simplex::edge(edge)).empty());
        REQUIRE(m.is_connectivity_valid());
    }
}

TEST_CASE("split_face", "[operations][split][2D]")
{
    using namespace operations::composite;
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
        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        auto result = op(Simplex::face(f));
        bool is_success = !result.empty();
        CHECK(is_success);
        const Tuple ret = result.front().tuple();
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(simplex::link(m, simplex::Simplex::vertex(ret)).simplex_vector().size() == 6);
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
        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        const auto res = op(Simplex::face(f));
        const bool is_success = !res.empty();
        CHECK(is_success);
        CHECK(m.get_all(PV).size() == 5);
        const auto ret_tuple = res.front().tuple();
        CHECK(m.id(ret_tuple, PV) == 5);
        CHECK(m.id(m.switch_vertex(ret_tuple), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret_tuple)), PV) == 0);
    }

    SECTION("split_with_attribute")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region_with_position();
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);

        MeshAttributeHandle<long> attri_handle =
            m.register_attribute<long>("test_attribute", PF, 1);

        MeshAttributeHandle<double> v2_handle = m.register_attribute<double>("vertices2", PV, 1);

        Accessor<long> acc_attri = m.create_accessor<long>(attri_handle);
        for (const Tuple& f : m.get_all(PF)) {
            acc_attri.scalar_attribute(f) = 1;
        }

        composite::TriFaceSplit op(m);

        {
            std::shared_ptr<wmtk::operations::SplitNewAttributeStrategy> new_split =
                std::make_shared<
                    wmtk::operations::tri_mesh::PredicateAwareSplitNewAttributeStrategy<double>>(
                    pos_handle);
            new_split->set_standard_split_strategy(
                operations::NewAttributeStrategy::SplitBasicStrategy::Default);
            new_split->set_standard_split_rib_strategy(
                operations::NewAttributeStrategy::SplitRibBasicStrategy::Default);
            op.split().set_strategy(pos_handle, new_split);
        }

        {
            // or if you want to swap out the existing behavior with a new behavior
            auto new_split = std::make_shared<
                wmtk::operations::tri_mesh::BasicSplitNewAttributeStrategy<double>>(pos_handle);
            new_split->set_standard_split_strategy(
                operations::NewAttributeStrategy::SplitBasicStrategy::Default);
            new_split->set_standard_split_rib_strategy(
                operations::NewAttributeStrategy::SplitRibBasicStrategy::Default);
            op.split().set_strategy(v2_handle, new_split);
        }

        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.split().set_standard_strategy(
            attri_handle,
            wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::Copy);
        op.collapse().set_standard_strategy(attri_handle);
        op.collapse().set_standard_strategy(pos_handle);
        op.collapse().set_standard_strategy(v2_handle);


        const Tuple f0 = m.face_tuple_from_vids(3, 4, 0);
        CHECK(!op(Simplex::face(f0)).empty());
        const Tuple f1 = m.face_tuple_from_vids(8, 9, 5);
        CHECK(!op(Simplex::face(f1)).empty());
        const Tuple f2 = m.face_tuple_from_vids(4, 8, 5);
        CHECK(!op(Simplex::face(f2)).empty());

        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_attri.scalar_attribute(t) == 1);
        }

        if (false) {
            io::Cache cache("");
            wmtk::io::ParaviewWriter writer(
                cache.get_cache_path() / "split_in_diamond_with_attribute_result",
                "vertices",
                m,
                false,
                false,
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
        MeshAttributeHandle<double> pos_handle = m.get_attribute_handle<double>("vertices", PV);

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.split().set_standard_strategy(pos_handle);
        op.collapse().set_standard_strategy(
            pos_handle,
            operations::NewAttributeStrategy::CollapseBasicStrategy::CopyOther);

        auto res = op(Simplex::face(f));
        bool is_success = !res.empty();
        CHECK(is_success);
        Tuple ret = res.front().tuple();
        CHECK(m.get_all(PV).size() == 4);
        CHECK(!m.is_boundary_vertex(ret));
        CHECK(!m.is_boundary_edge(ret));
        CHECK(!m.is_boundary_edge(m.switch_edge(ret)));
        CHECK(m.id(ret, PV) == 4);
        CHECK(m.id(m.switch_vertex(ret), PV) == 1);
        CHECK(m.id(m.switch_vertex(m.switch_edge(ret)), PV) == 2);
        CHECK(simplex::vertex_one_ring(m, ret).size() == 3);
        Accessor<double> acc_pos = m.create_accessor<double>(pos_handle);
        CHECK(acc_pos.vector_attribute(ret).x() == 0.375);
        CHECK(acc_pos.vector_attribute(m.switch_vertex(ret)).x() == 1);
    }
    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_on")
    {
        //    0---1---2
        //   / \ / \ / \ .
        //  3---4---5---6
        //   \ / \ / \ /
        //    7---8---9
        DEBUG_TriMesh m = edge_region();
        Tuple f = m.edge_tuple_between_v1_v2(3, 4, 0);

        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);

        Accessor<long> acc_todo = m.create_accessor<long>(todo_handle);
        acc_todo.scalar_attribute(f) = 1;

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));
        op.split().set_standard_strategy(
            todo_handle,
            NewAttributeStrategy::SplitBasicStrategy::None,
            NewAttributeStrategy::SplitRibBasicStrategy::None);
        op.collapse().set_standard_strategy(
            todo_handle,
            NewAttributeStrategy::CollapseBasicStrategy::None);

        CHECK(!op(Simplex::face(f)).empty());

        CHECK(m.get_all(PF).size() == 12);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }

    SECTION("split_single_triangle_at_mid_point_with_tag_embedding_off")
    {
        DEBUG_TriMesh m = single_triangle();

        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);
        MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>("edge_tag", PE, 1);
        MeshAttributeHandle<long> vertex_tag_handle =
            m.register_attribute<long>("vertex_tag", PV, 1);
        Accessor<long> acc_todo = m.create_accessor<long>(todo_handle);
        Accessor<long> acc_edge_tag = m.create_accessor<long>(edge_tag_handle);
        Accessor<long> acc_vertex_tag = m.create_accessor<long>(vertex_tag_handle);
        acc_todo.scalar_attribute(f) = 1;

        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 1;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 2;
        acc_edge_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;

        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(0, 1, 0)) = 1;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(1, 2, 0)) = 2;
        acc_vertex_tag.scalar_attribute(m.edge_tuple_between_v1_v2(2, 0, 0)) = 3;

        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));

        op.split().set_standard_strategy(
            todo_handle,
            NewAttributeStrategy::SplitBasicStrategy::None,
            NewAttributeStrategy::SplitRibBasicStrategy::None);
        op.collapse().set_standard_strategy(
            todo_handle,
            NewAttributeStrategy::CollapseBasicStrategy::None);

        op.split().set_standard_strategy(
            edge_tag_handle,
            wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::Copy,
            wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy::None);
        op.collapse().set_standard_strategy(
            edge_tag_handle,
            wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy::None);

        op.split().set_standard_strategy(
            vertex_tag_handle,
            wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::None,
            wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy::None);
        op.collapse().set_standard_strategy(
            vertex_tag_handle,
            wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy::None);


        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));
        const auto res = op(Simplex::face(f));

        CHECK(!res.empty());
        const Tuple& return_tuple = res.front().tuple();
        const Tuple v0 = m.switch_vertex(m.switch_edge(m.switch_face(return_tuple)));
        const Tuple v1 = m.switch_vertex(return_tuple);
        const Tuple v2 = m.switch_vertex(m.switch_edge(return_tuple));

        const long id0 = m.id_vertex(return_tuple);
        const long id1 = m.id_vertex(m.switch_vertex(return_tuple));
        const long id2 = m.id_vertex(m.switch_vertex(m.switch_edge(return_tuple)));
        CHECK(acc_vertex_tag.scalar_attribute(return_tuple) == 0);
        CHECK(acc_vertex_tag.scalar_attribute(v0) == 1);
        CHECK(acc_vertex_tag.scalar_attribute(v1) == 2);
        CHECK(acc_vertex_tag.scalar_attribute(v2) == 3);

        CHECK(acc_edge_tag.scalar_attribute(return_tuple) == 0);

        const Tuple e01 = m.switch_edge(m.switch_face(m.switch_vertex(return_tuple)));
        const Tuple e12 = m.switch_edge(m.switch_vertex(return_tuple));
        const Tuple e20 =
            m.switch_edge(m.switch_face(m.switch_vertex(m.switch_edge(return_tuple))));
        CHECK(acc_edge_tag.scalar_attribute(e01) == 1);
        CHECK(acc_edge_tag.scalar_attribute(e12) == 2);
        CHECK(acc_edge_tag.scalar_attribute(e20) == 3);

        CHECK(m.get_all(PF).size() == 3);
        for (const Tuple& t : m.get_all(PF)) {
            CHECK(acc_todo.scalar_attribute(t) == 0);
        }
    }

    SECTION("should fail with todo tag 0")
    {
        DEBUG_TriMesh m = single_equilateral_triangle(3);

        const Tuple f = m.edge_tuple_between_v1_v2(1, 2, 0);
        MeshAttributeHandle<long> todo_handle = m.register_attribute<long>("todo_face", PF, 1);


        composite::TriFaceSplit op(m);
        op.collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(m));
        op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));

        op.split().set_standard_strategy(
            todo_handle,
            NewAttributeStrategy::SplitBasicStrategy::None,
            NewAttributeStrategy::SplitRibBasicStrategy::None);

        CHECK(op(Simplex::face(f)).empty());
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
    wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);

    wmtk::MeshAttributeHandle<long> edge_tag_handle = m.register_attribute<long>("edge_tag", PE, 1);
    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    SECTION("single_split")
    {
        EdgeSplit op(m);

        op.set_standard_strategy(
            edge_tag_handle,
            NewAttributeStrategy::SplitBasicStrategy::Copy,
            NewAttributeStrategy::SplitRibBasicStrategy::None);
        op.set_standard_strategy(pos_handle);

        const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);
        {
            auto acc_tag_e = m.create_accessor(edge_tag_handle);
            acc_tag_e.scalar_attribute(t) = 1;
        }

        const auto res = op(Simplex::edge(t));
        CHECK(!res.empty());

        const Tuple spine1 = res.front().tuple();
        const Tuple rib1 = m.switch_edge(m.switch_face(spine1));
        const Tuple spine2 = m.switch_edge(m.switch_face(rib1));
        const Tuple rib2 = m.switch_edge(m.switch_face(spine2));
        {
            auto acc_tag_e = m.create_accessor(edge_tag_handle);
            CHECK(acc_tag_e.scalar_attribute(spine1) == 1);
            CHECK(acc_tag_e.scalar_attribute(spine2) == 1);
            CHECK(acc_tag_e.scalar_attribute(rib1) == 0);
            CHECK(acc_tag_e.scalar_attribute(rib2) == 0);
        }
    }

    wmtk::MeshAttributeHandle<long> vertex_tag_handle =
        m.register_attribute<long>(std::string("vertex_tag"), PV, 1);

    wmtk::MeshAttributeHandle<long> todo_handle =
        m.register_attribute<long>(std::string("todo_tag"), PE, 1);

    EdgeSplit op(m);
    op.set_standard_strategy(pos_handle);
    op.set_standard_strategy(vertex_tag_handle);
    op.set_standard_strategy(
        edge_tag_handle,
        NewAttributeStrategy::SplitBasicStrategy::Copy,
        NewAttributeStrategy::SplitRibBasicStrategy::None);

    op.set_standard_strategy(
        todo_handle,
        wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::None,
        wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy::None);


    op.add_invariant(std::make_shared<TodoInvariant>(m, todo_handle));

    SECTION("no_todo_edges")
    {
        for (const Tuple& t : m.get_all(PV)) {
            CHECK(op(Simplex::edge(t)).empty());
        }
    }

    SECTION("interior_todo_with_tags")
    {
        wmtk::Accessor<long> acc_todo = m.create_accessor(todo_handle);
        wmtk::Accessor<long> acc_tag_e = m.create_accessor(edge_tag_handle);
        wmtk::Accessor<long> acc_tag_v = m.create_accessor(vertex_tag_handle);
        for (const Tuple& e : m.get_all(PE)) {
            if (!m.is_boundary_edge(e)) {
                acc_tag_e.scalar_attribute(e) = 1;
                acc_todo.scalar_attribute(e) = 1;
            }
        }
        int success_num = 0;
        // should perform two iterations to split the two interior edges once
        for (int i = 0; i < 5; i++) {
            for (const Tuple& t : m.get_all(PE)) {
                const auto res = op(Simplex::edge(t));
                if (!res.empty()) {
                    const Tuple spine1 = res.front().tuple();
                    const Tuple rib1 = m.switch_edge(m.switch_face(spine1));
                    const Tuple spine2 = m.switch_edge(m.switch_face(rib1));
                    const Tuple rib2 = m.switch_edge(m.switch_face(spine2));
                    // todo marks should be removed
                    CHECK(acc_todo.scalar_attribute(spine1) == 0);
                    CHECK(acc_todo.scalar_attribute(spine2) == 0);
                    CHECK(acc_todo.scalar_attribute(rib1) == 0);
                    CHECK(acc_todo.scalar_attribute(rib2) == 0);

                    // splitted edges should be tagged
                    CHECK(acc_tag_e.scalar_attribute(spine1) == 1);
                    CHECK(acc_tag_e.scalar_attribute(spine2) == 1);
                    // new edges should not be tagged
                    CHECK(acc_tag_e.scalar_attribute(rib1) == 0);
                    CHECK(acc_tag_e.scalar_attribute(rib2) == 0);

                    // TODO test for vertex attributes

                    success_num++;
                }
            }
        }
        CHECK(success_num == 2);
    }
}
