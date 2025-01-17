
#include <fmt/ranges.h>
#include <bitset>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <tools/single_simplex_mesh.hpp>
#include <wmtk/autogen/utils/largest_shared_subdart_size.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>
#include <wmtk/operations/internal/CollapseAlternateFacetData.hpp>
#include <wmtk/operations/internal/SplitAlternateFacetData.hpp>
#include <wmtk/operations/internal/ear_actions.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/global_ids.hpp"
#include "../tools/global_index_max_subdart_size.hpp"
#include "tools/DEBUG_Mesh.hpp"


TEST_CASE("split_facet_maps", "[operations][data]")
{
    for (wmtk::PrimitiveType mesh_type : wmtk::utils::primitive_range(
             wmtk::PrimitiveType::Edge,
             wmtk::PrimitiveType::Tetrahedron)) {
        wmtk::autogen::SimplexDart sd(mesh_type);
        wmtk::operations::internal::SplitAlternateFacetData data;

        auto& scm = data.m_facet_maps;


        auto add = [&](int64_t index, int8_t s, const std::array<int64_t, 2>& pr) {
            scm.emplace_back(wmtk::autogen::Dart(index, s % sd.size()), pr);
        };

        add(0, 2, std::array<int64_t, 2>{{1, 2}});
        add(4, 20, std::array<int64_t, 2>{{9, 4}});
        add(3, 30, std::array<int64_t, 2>{{8, 5}});
        add(2, 5, std::array<int64_t, 2>{{10, 3}});


        data.sort();

        CHECK(scm[0].input.global_id() == 0);
        CHECK(scm[1].input.global_id() == 2);
        CHECK(scm[2].input.global_id() == 3);
        CHECK(scm[3].input.global_id() == 4);

        CHECK(data.get_alternative_facets(0) == std::array<int64_t, 2>{{1, 2}});
        CHECK(data.get_alternative_facets(4) == std::array<int64_t, 2>{{9, 4}});
        CHECK(data.get_alternative_facets(3) == std::array<int64_t, 2>{{8, 5}});
        CHECK(data.get_alternative_facets(2) == std::array<int64_t, 2>{{10, 3}});

        CHECK(data.get_alternative_facets_it(1) == scm.cend());
        CHECK(data.get_alternative_facets_it(6) == scm.cend());
        CHECK(data.get_alternative_facets_it(7) == scm.cend());


        const int8_t LA = wmtk::operations::internal::left_ear_action(mesh_type);
        const int8_t RA = wmtk::operations::internal::right_ear_action(mesh_type);
        const std::array<int8_t, 2> actions{{LA, RA}};

        const wmtk::PrimitiveType boundary_type = mesh_type - 1;
        for (const auto& scm_data : scm) {
            std::array<int8_t, 2> boundaries;
            for (size_t j = 0; j < 2; ++j) {
                boundaries[j] = sd.simplex_index(
                    sd.product(actions[j], scm_data.input.local_orientation()),
                    boundary_type);
            }
            for (int8_t j = 0; j < sd.size(); ++j) {
                int left_efficacy = wmtk::autogen::utils::largest_shared_subdart_size(
                    mesh_type,
                    j,
                    boundary_type,
                    boundaries[0]);
                int right_efficacy = wmtk::autogen::utils::largest_shared_subdart_size(
                    mesh_type,
                    j,
                    boundary_type,
                    boundaries[1]);
                // CHECK(
                //     left_efficacy ==
                // CHECK(
                //     right_efficacy ==

                if (left_efficacy <= 0 && right_efficacy <= 0) {
                    continue;
                } else {
                    int64_t new_gid = scm_data.new_gid(mesh_type, j);
                    spdlog::info(
                        "{} ({} {}), {}",
                        fmt::join(scm_data.new_facet_indices, ","),
                        left_efficacy,
                        right_efficacy,
                        new_gid);
                    if (left_efficacy > right_efficacy) {
                        CHECK(new_gid == scm_data.new_facet_indices[0]);
                    } else if (left_efficacy < right_efficacy) {
                        CHECK(new_gid == scm_data.new_facet_indices[1]);
                    } else {
                        const bool either = new_gid == scm_data.new_facet_indices[0] ||
                                            new_gid == scm_data.new_facet_indices[1];
                        CHECK(either);
                    }
                }

                {
                    // test against a mesh
                }
            }
        }
    }

    // CHECK(data.get_alternate_dart());
}

TEST_CASE("split_facet_maps_mesh", "[operations][data]")
{
    for (wmtk::PrimitiveType mesh_type : wmtk::utils::primitive_range(
             wmtk::PrimitiveType::Edge,
             wmtk::PrimitiveType::Tetrahedron)) {
        wmtk::autogen::SimplexDart sd(mesh_type);


        const int8_t LA = wmtk::operations::internal::left_ear_action(mesh_type);
        const int8_t RA = wmtk::operations::internal::right_ear_action(mesh_type);
        const std::array<int8_t, 2> actions{{LA, RA}};

        const wmtk::PrimitiveType boundary_type = mesh_type - 1;
        for (int8_t edge_orientation = 0; edge_orientation < sd.size(); ++edge_orientation) {
            auto mesh_ptr = wmtk::tests::tools::single_simplex_mesh(mesh_type);
            spdlog::info("Mesh dimension: {}", mesh_ptr->top_cell_dimension());

            wmtk::Tuple t = sd.tuple_from_dart(wmtk::autogen::Dart(0, edge_orientation));
            // data.add(*mesh_ptr, t);
            //  std::array<int8_t, 2> boundaries;
            //   for (size_t j = 0; j < 2; ++j) {
            //       boundaries[j] = sd.simplex_index(
            //           sd.product(actions[j], scm_data.input.local_orientation()),
            //           boundary_type);
            //   }


            const auto original_global_ids = wmtk::tests::tools::global_ids(*mesh_ptr, t);

            wmtk::operations::utils::MultiMeshEdgeSplitFunctor split;
            auto data = split.run(*mesh_ptr, wmtk::simplex::Simplex::edge(*mesh_ptr, t));
            const auto& split_data = data.const_split_facet_data();

            REQUIRE(split_data.m_facet_maps.size() == 1);
            const auto& scm_data = split_data.m_facet_maps[0];

            wmtk::Tuple left_tuple = sd.tuple_from_dart(
                wmtk::autogen::Dart(scm_data.new_facet_indices[0], edge_orientation));
            wmtk::Tuple right_tuple = sd.tuple_from_dart(
                wmtk::autogen::Dart(scm_data.new_facet_indices[1], edge_orientation));
            spdlog::info(
                "{} {}",
                wmtk::utils::TupleInspector::as_string(left_tuple),
                wmtk::utils::TupleInspector::as_string(right_tuple));

            const auto left_global_ids = wmtk::tests::tools::global_ids(*mesh_ptr, left_tuple);
            const auto right_global_ids = wmtk::tests::tools::global_ids(*mesh_ptr, right_tuple);

            spdlog::info(
                "{}:{} {}:{}",
                wmtk::utils::TupleInspector::as_string(left_tuple),
                fmt::join(left_global_ids, ","),
                wmtk::utils::TupleInspector::as_string(right_tuple),
                fmt::join(right_global_ids, ","));

            int8_t left_size = wmtk::tests::tools::global_index_max_subdart_size(
                original_global_ids,
                left_global_ids);
            int8_t right_size = wmtk::tests::tools::global_index_max_subdart_size(
                original_global_ids,
                right_global_ids);

            // CHECK(
            //     left_efficacy ==
            // CHECK(
            //     right_efficacy ==
        }
    }

    // CHECK(data.get_alternate_dart());
}

namespace {
void collapse_facet_maps_impl(
    wmtk::operations::internal::CollapseAlternateFacetData& data,
    const wmtk::Mesh& m,
    const std::vector<std::tuple<wmtk::Tuple, std::bitset<2>>>& tuples_with_boundary_info)
{
    for (const auto& [tuple, bits] : tuples_with_boundary_info) {
        spdlog::info("Adding {}", wmtk::utils::TupleInspector::as_string(tuple));
        data.add(m, tuple);
    }

    //
}
} // namespace

TEST_CASE("collapse_facet_maps_1d", "[operations][data][1D][.]")
{
    {
        spdlog::info("1d");
        wmtk::operations::internal::CollapseAlternateFacetData data;


        auto m = wmtk::tests::multiple_lines(4);
        auto& m_debug = reinterpret_cast<wmtk::tests::DEBUG_Mesh&>(m);
        std::vector<std::tuple<wmtk::Tuple, std::bitset<2>>> bdata;

        for (const auto& t : m.get_all(wmtk::PrimitiveType::Edge)) {
            bdata.emplace_back(
                t,
                1 * int(m.is_boundary(wmtk::PrimitiveType::Vertex, t)) +
                    2 * int(m.is_boundary(wmtk::PrimitiveType::Vertex, m.switch_vertex(t))));
        }

        collapse_facet_maps_impl(data, m, bdata);

        {
            const auto& data_vec = data.m_data;
            REQUIRE(data_vec.size() == bdata.size());
            for (size_t j = 0; j < data_vec.size(); ++j) {
                const auto& d = data_vec[j];
                const auto& b = bdata[j];
                const auto& bits = std::get<1>(b);
                for (size_t k = 0; k < 2; ++k) {
                    const auto& alt = d.alts[k];
                    REQUIRE(alt.is_null() == bits[k]);
                     const auto& i = d.local_boundary_indices[k];
                     spdlog::info("Bdata {}:{} global id {}  with local orientation {} => points to {}", j, k, alt.global_id(), alt.local_orientation(), i);
                }
            }
        }

        for (const auto& [t, bits] : bdata) {
            // make sure there were alternatives to begin with

            int64_t index = m_debug.id(t, m.top_simplex_type());
            spdlog::info("Testing on {}", index);
            REQUIRE(bits != 3);
            auto both = data.get_alternatives(m.top_simplex_type(), t, wmtk::PrimitiveType::Vertex);
            auto [a, b] = both;
            static_assert(std::is_same_v<std::decay_t<decltype(a)>, wmtk::Tuple>);
            static_assert(std::is_same_v<std::decay_t<decltype(b)>, wmtk::Tuple>);
            {
                // just check that it's not null and that it's one of the two optiosn
                const wmtk::Tuple new_tup =
                    data.get_alternative(m.top_simplex_type(), t, wmtk::PrimitiveType::Vertex);
                CHECK(!new_tup.is_null());
                CHECK((new_tup == a || new_tup == b));
            }
            for (size_t j = 0; j < 2; ++j) {
                int64_t sign = j * 2 - 1;
                size_t nbr_edge_index = index + sign;
                size_t vertex_index = index + j;

                const auto& tup = both[j];
                // boundary == bits is 1
                 //spdlog::info(
                 //   "{}: {} => {}",
                 //   j,
                 //   wmtk::utils::TupleInspector::as_string(tup),
                    //bits[j]);
                REQUIRE(tup.is_null() == bits[j]);
                if (!bits[j]) { // not boundary
                    // check that the tuple returned makes sense
                    REQUIRE(m.is_valid(tup));
                    CHECK(m_debug.id(tup, wmtk::PrimitiveType::Vertex) == vertex_index);
                    CHECK(m_debug.id(tup, wmtk::PrimitiveType::Edge) == nbr_edge_index);
                }
            }
        }
    }
}
TEST_CASE("collapse_facet_maps_2d", "[operations][data][2D][.]")
{
    {
        spdlog::info("2d");
        {
            wmtk::operations::internal::CollapseAlternateFacetData data;


            auto m = wmtk::tests::two_neighbors();
            wmtk::autogen::SimplexDart sd(m.top_simplex_type());
            auto& m_debug = reinterpret_cast<wmtk::tests::DEBUG_Mesh&>(m);
            auto& m_tri_debug = reinterpret_cast<wmtk::tests::DEBUG_TriMesh&>(m);


            // template for the following:
            //   x-----x-----x
            //   |    / \    |
            //   |   /   \   |
            //   |  /     \  |
            //   | /       \ |
            //   x-----------x

            // estimate of the edge used for teh collapse
            //   x-----x-----x
            //   |    / \    |
            //   |   /   \   |
            //   |  /  o  \  |
            //   | /       \ |
            //   o-----o-----x
            wmtk::Tuple main_tuple = m_tri_debug.tuple_from_global_ids(0, 4, 1);
            const auto main_dart = sd.dart_from_tuple(main_tuple);

            data.add(m, main_tuple);


            // the input edge's "mirror" to access ears easier
            // collapsing this edge does the same topological operation
            //   x-----x-----x
            //   |    / \    |
            //   |   /   \   |
            //   |  /  o  \  |
            //   | /       \ |
            //   x-----o-----o
            wmtk::Tuple dual_tuple = m_tri_debug.tuple_from_global_ids(0, 4, 2);
            REQUIRE(m.switch_vertex(main_tuple) == dual_tuple);

            // tuples in the input face for the left ear's edge
            //   x-----x-----x
            //   |    / \    |
            //   |   o   \   |
            //   |  /  o  \  |
            //   | /       \ |
            //   o-----------x
            wmtk::Tuple left_ear = m_tri_debug.tuple_from_global_ids(0, 0, 1);
            REQUIRE(m.switch_edge(main_tuple) == left_ear);
            //   x-----o-----x
            //   |    / \    |
            //   |   o   \   |
            //   |  /  o  \  |
            //   | /       \ |
            //   x-----------x
            wmtk::Tuple left_ear_opp = m_tri_debug.tuple_from_global_ids(0, 0, 0);
            const auto left_ear_opp_dart = sd.dart_from_tuple(left_ear_opp);
            REQUIRE(m.switch_vertex(left_ear) == left_ear_opp);


            // tuples in the input face for the right ear's edge
            //   x-----x-----x
            //   |    / \    |
            //   |   /   o   |
            //   |  /  o  \  |
            //   | /       \ |
            //   x-----------o
            wmtk::Tuple right_ear = m_tri_debug.tuple_from_global_ids(0, 1, 2);
            REQUIRE(m.switch_edge(dual_tuple) == right_ear);
            //   x-----o-----x
            //   |    / \    |
            //   |   /   o   |
            //   |  /  o  \  |
            //   | /       \ |
            //   x-----------x
            wmtk::Tuple right_ear_opp = m_tri_debug.tuple_from_global_ids(0, 1, 0);
            const auto right_ear_opp_dart = sd.dart_from_tuple(right_ear_opp);
            REQUIRE(m.switch_vertex(right_ear) == right_ear_opp);


            // left ear's tuples but in triangle 1 (should have equivalent vertex and edge
            //   x-----x-----x
            //   | o  / \    |
            //   |   o   \   |
            //   |  /     \  |
            //   | /       \ |
            //   o-----------x
            wmtk::Tuple left_alt = m_tri_debug.tuple_from_global_ids(1, 0, 1);
            const auto left_alt_dart = sd.dart_from_tuple(left_alt);
            REQUIRE(m.switch_face(left_ear) == left_alt);
            //   x-----o-----x
            //   | o  / \    |
            //   |   o   \   |
            //   |  /     \  |
            //   | /       \ |
            //   x-----------x
            wmtk::Tuple left_alt_opp = m_tri_debug.tuple_from_global_ids(1, 0, 0);
            const auto left_alt_opp_dart = sd.dart_from_tuple(left_alt_opp);
            REQUIRE(m.switch_vertex(left_alt) == left_alt_opp);
            REQUIRE(m.switch_face(left_ear_opp) == left_alt_opp);

            //   x-----x-----x
            //   |    / \  o |
            //   |   /   o   |
            //   |  /     \  |
            //   | /       \ |
            //   x-----------o
            wmtk::Tuple right_alt = m_tri_debug.tuple_from_global_ids(2, 1, 2);
            const auto right_alt_dart = sd.dart_from_tuple(right_alt);
            REQUIRE(m.switch_face(right_ear) == right_alt);
            //   x-----o-----x
            //   |    / \ o  |
            //   |   /   o   |
            //   |  /     \  |
            //   | /       \ |
            //   x-----------x
            wmtk::Tuple right_alt_opp = m_tri_debug.tuple_from_global_ids(2, 1, 0);
            const auto right_alt_opp_dart = sd.dart_from_tuple(right_alt_opp);
            REQUIRE(m.switch_vertex(right_alt) == right_alt_opp);
            REQUIRE(m.switch_face(right_ear_opp) == right_alt_opp);


            {
                // premable debug printouts to check what happened in add
                fmt::print(
                    "Should have a main {} => left opp {} == left alt {}\n",
                    std::string(main_dart),
                    std::string(left_ear_opp_dart),
                    std::string(left_alt_opp_dart));
                fmt::print(
                    "Should have a main {} => right opp {} == right alt {}\n",
                    std::string(main_dart),
                    std::string(right_ear_opp_dart),
                    std::string(right_alt_opp_dart));

                const auto& data_vec = data.m_data;
                REQUIRE(data_vec.size() == 1);
                const auto& dat = data_vec[0];

                const auto& left_dart = dat.alts[0];
                const auto& right_dart = dat.alts[1];
                const int8_t left_ear_eid = wmtk::utils::TupleInspector::local_eid(left_ear);
                const int8_t right_ear_eid = wmtk::utils::TupleInspector::local_eid(right_ear);
                CHECK(left_ear_eid == dat.local_boundary_indices[0]);
                CHECK(right_ear_eid == dat.local_boundary_indices[1]);

                CHECK(left_dart.global_id() == 1);
                CHECK(right_dart.global_id() == 2);

                auto left_act = sd.act(main_dart, left_dart.local_orientation());
                auto right_act = sd.act(main_dart, right_dart.local_orientation());
                fmt::print(
                    "{} => {} {} == {} {} (ignore the global ids)\n",
                    std::string(main_dart),
                    std::string(left_alt_opp_dart),
                    std::string(right_alt_opp_dart),
                    std::string(left_act),
                    std::string(right_act));
                CHECK(left_alt_opp_dart.local_orientation() == left_act.local_orientation());
                CHECK(right_alt_opp_dart.local_orientation() == right_act.local_orientation());
            }
            std::vector<std::tuple<wmtk::Tuple, wmtk::Tuple>> left_alternatives, right_alternatives;

            left_alternatives.emplace_back(left_ear, left_alt);
            left_alternatives.emplace_back(left_ear_opp, left_alt_opp);

            right_alternatives.emplace_back(right_ear, right_alt);
            right_alternatives.emplace_back(right_ear_opp, right_alt_opp);

            // left result, right result, expected subdart preservation
            using Dat = std::tuple<wmtk::Tuple, wmtk::Tuple, wmtk::PrimitiveType>;
            std::vector<std::tuple<wmtk::Tuple, Dat>> results;

            results.emplace_back(left_ear, Dat{left_alt, {}, wmtk::PrimitiveType::Edge});
            results.emplace_back(right_ear, Dat{{}, right_alt, wmtk::PrimitiveType::Edge});
            // results.emplace_back(
            //     left_ear_opp,
            //     std::array<wmtk::Tuple, 2>{{left_alt_opp, right_alt_opp}});
            // results.emplace_back(
            //     right_ear_opp,
            //     std::array<wmtk::Tuple, 2>{{left_alt_opp, right_alt_opp}});


            for (const auto& [t, pr] : results) {
                auto ret =
                    data.get_alternatives(m.top_simplex_type(), t, wmtk::PrimitiveType::Edge);

                const auto& [a, b, pt] = pr;
                const auto& [c, d] = ret;
                spdlog::info(
                    "Input {}: Expecteed two alts{} {} => Got two alts{} {}",
                    wmtk::utils::TupleInspector::as_string(t),
                    wmtk::utils::TupleInspector::as_string(a),
                    wmtk::utils::TupleInspector::as_string(b),
                    wmtk::utils::TupleInspector::as_string(c),
                    wmtk::utils::TupleInspector::as_string(d));
                // notation is triangle; vertex, edge (matches global; local vid, local eid)
                // 0; 1,2 (global: 0; 1,0)
                // currently: 1;1,0 2;1,2 => 1;1,2 2;1,0
                CHECK(a == c);
                CHECK(b == d);
            }
        }
    }
}
