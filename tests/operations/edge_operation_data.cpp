
#include <catch2/catch_test_macros.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>


TEST_CASE("split_cell_maps", "[operations][data]")
{
    wmtk::operations::EdgeOperationData data;

    auto& scm = data.m_split_cell_maps;

    scm.emplace_back(std::tuple{0, std::array<int64_t, 2>{{1, 2}}});
    scm.emplace_back(std::tuple{4, std::array<int64_t, 2>{{9, 4}}});
    scm.emplace_back(std::tuple{3, std::array<int64_t, 2>{{8, 5}}});
    scm.emplace_back(std::tuple{2, std::array<int64_t, 2>{{10, 3}}});


    data.sort_split_cell_map();

    CHECK(std::get<0>(scm[0]) == 0);
    CHECK(std::get<0>(scm[1]) == 2);
    CHECK(std::get<0>(scm[2]) == 3);
    CHECK(std::get<0>(scm[3]) == 4);

    CHECK(data.get_split_output_cells(0) == std::array<int64_t, 2>{{1, 2}});
    CHECK(data.get_split_output_cells(4) == std::array<int64_t, 2>{{9, 4}});
    CHECK(data.get_split_output_cells(3) == std::array<int64_t, 2>{{8, 5}});
    CHECK(data.get_split_output_cells(2) == std::array<int64_t, 2>{{10, 3}});
}
