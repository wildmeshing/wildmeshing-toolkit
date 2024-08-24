
#include <bitset>
#include <catch2/catch_test_macros.hpp>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>
#include <wmtk/operations/internal/CollapseAlternateFacetData.hpp>
#include <wmtk/operations/internal/SplitAlternateFacetData.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "../tools/EdgeMesh_examples.hpp"
#include "tools/DEBUG_Mesh.hpp"


TEST_CASE("split_facet_maps", "[operations][data]")
{
    wmtk::operations::internal::SplitAlternateFacetData data;

    auto& scm = data.m_facet_maps;

    scm.emplace_back(std::tuple{0, std::array<int64_t, 2>{{1, 2}}});
    scm.emplace_back(std::tuple{4, std::array<int64_t, 2>{{9, 4}}});
    scm.emplace_back(std::tuple{3, std::array<int64_t, 2>{{8, 5}}});
    scm.emplace_back(std::tuple{2, std::array<int64_t, 2>{{10, 3}}});


    data.sort();

    CHECK(std::get<0>(scm[0]) == 0);
    CHECK(std::get<0>(scm[1]) == 2);
    CHECK(std::get<0>(scm[2]) == 3);
    CHECK(std::get<0>(scm[3]) == 4);

    CHECK(data.get_alternative_facets(0) == std::array<int64_t, 2>{{1, 2}});
    CHECK(data.get_alternative_facets(4) == std::array<int64_t, 2>{{9, 4}});
    CHECK(data.get_alternative_facets(3) == std::array<int64_t, 2>{{8, 5}});
    CHECK(data.get_alternative_facets(2) == std::array<int64_t, 2>{{10, 3}});

    CHECK(data.get_alternative_facets_it(1) == scm.cend());
    CHECK(data.get_alternative_facets_it(6) == scm.cend());
    CHECK(data.get_alternative_facets_it(7) == scm.cend());

    //CHECK(data.get_alternate_dart());
}


namespace {
void collapse_facet_maps_impl(
    wmtk::operations::internal::CollapseAlternateFacetData& data,
    const wmtk::Mesh& m,
    const std::vector<std::tuple<wmtk::Tuple, std::bitset<2>>>& tuples_with_boundary_info)
{
    for (const auto& [tuple, bits] : tuples_with_boundary_info) {
        data.add(m, tuple);
    }

    //
}
} // namespace

TEST_CASE("collapse_facet_maps_1d", "[operations][data][1D]")
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
                for (size_t j = 0; j < 2; ++j) {
                    const auto& alt = d.alts[j];
                    REQUIRE(alt.is_null() == bits[j]);
                    // const auto& i = d.local_boundary_indices[j];
                    // spdlog::info("{} {} => {}", alt.global_id(), alt.local_orientation(), i);
                }
            }
        }

        for (const auto& [t, bits] : bdata) {
            // make sure there were alternatives to begin with

            int64_t index = m_debug.id(t, m.top_simplex_type());
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
                // spdlog::info(
                //    "{}: {} => {}",
                //    j,
                //    wmtk::utils::TupleInspector::as_string(tup),
                //    bits[j]);
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
TEST_CASE("collapse_facet_maps_2d", "[operations][data][2D]")
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
