#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/extract_subset/extract_subset.hpp>
#include <wmtk_components/extract_subset/internal/extract_subset_2d.hpp>
#include "../tools/TriMesh_examples.hpp"

TEST_CASE("2d_tetrahedron_test_case", "[components][extract_subset][2D]")
{
    wmtk::TriMesh tm = wmtk::tests::tetrahedron_with_position();
    std::vector<int> tag_vector = {1, 0, 1, 1};
    wmtk::TriMesh new_tm = wmtk::components::extract_subset(tm, 2, tag_vector, true);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Face) == 3);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Vertex) == 4);
    CHECK(new_tm.capacity(wmtk::PrimitiveType::Edge) == 6);

    wmtk::ParaviewWriter writer("mesh_smooth", "vertices", new_tm, true, true, true, false);
    new_tm.serialize(writer);
}