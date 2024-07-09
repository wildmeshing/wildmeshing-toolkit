
#include <bitset>
#include <catch2/catch_test_macros.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>
#include "../tools/EdgeMesh_examples.hpp"
#include "tools/DEBUG_Mesh.hpp"


TEST_CASE("split_facet_maps", "[operations][data]")
{
    wmtk::operations::SplitAlternateFacetData data;

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
}


namespace {
void collapse_facet_maps_impl(
    wmtk::operations::CollapseAlternateFacetData& data,
    const wmtk::Mesh& m,
    const std::vector<std::tuple<wmtk::Tuple, std::bitset<2>>>& tuples_with_boundary_info)
{
    for (const auto& [tuple, bits] : tuples_with_boundary_info) {
        data.add(m, tuple);
    }

    //
}
} // namespace

TEST_CASE("collapse_facet_maps_2d", "[operations][data][2D]")
{
    wmtk::operations::CollapseAlternateFacetData data;


    auto m = wmtk::tests::multiple_lines(4);
    auto& m_debug = reinterpret_cast<wmtk::tests::DEBUG_Mesh&>(m);
    std::vector<std::tuple<wmtk::Tuple, std::bitset<2>>> bdata;

    for (const auto& t : m.get_all(wmtk::PrimitiveType::Edge)) {
        bdata.emplace_back(
            t,
            2 * int(!m.is_boundary(wmtk::PrimitiveType::Vertex, t)) +
                int(!m.is_boundary(wmtk::PrimitiveType::Vertex, m.switch_vertex(t))));
    }

    collapse_facet_maps_impl(data, m, bdata);

    for (const auto& [t, bits] : bdata) {
        // make sure there were alternatives to begin with

        int64_t index = m_debug.id(t, m.top_simplex_type());
        REQUIRE(bits != 0);
        auto both = data.get_alternatives(m.top_simplex_type(), t);
        auto [a, b] = both;
        static_assert(std::is_same_v<std::decay_t<decltype(a)>, wmtk::Tuple>);
        static_assert(std::is_same_v<std::decay_t<decltype(b)>, wmtk::Tuple>);
        {
            // just check that it's not null and that it's one of the two optiosn
            const wmtk::Tuple new_tup = data.get_alternative(m.top_simplex_type(), t);
            CHECK(!new_tup.is_null());
            CHECK((new_tup == a || new_tup == b));
        }
        for (size_t j = 0; j < 2; ++j) {
            int64_t sign = j * 2 - 1;
            size_t nbr_edge_index = index + sign;
            size_t vertex_index = index + j;
            if (bits[j]) {
                // check that the tuple returned makes sense
                const auto& tup = both[j];
                CHECK(m_debug.id(tup, wmtk::PrimitiveType::Vertex) == vertex_index);
                CHECK(m_debug.id(tup, wmtk::PrimitiveType::Edge) == nbr_edge_index);
            } else {
                CHECK(both[j].is_null());
            }
        }
    }
}
