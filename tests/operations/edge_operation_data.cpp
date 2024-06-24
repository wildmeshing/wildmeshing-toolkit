
#include <catch2/catch_test_macros.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>


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
