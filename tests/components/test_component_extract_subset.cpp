#include <wmtk_components/extract_subset/internal/extract_subset_2d.hpp>
#include <wmtk_components/extract_subset/extract_subset.hpp>
#include <catch2/catch_test_macros.hpp>
#include "../tools/TriMesh_examples.hpp"
#include "wmtk/attribute/Attribute.hpp"
#include "wmtk/Primitive.hpp"
#include "wmtk/attribute/AttributeHandle.hpp"
#include "wmtk/Mesh.hpp"

TEST_CASE("3 neighbors test case", "[components][extract_subset][2D]")
{
    wmtk::TriMesh tm;
    tm = wmtk::tests::three_neighbors(); // start with 4 tri
    Eigen::Vector<long, 10> V; // init the data vector with any static length, then resize
    V.resize(tm.capacity( wmtk::PrimitiveType::Face), 1);
    V.row(0) << 1;
    V.row(1) << 0; // ont tri is not tagged
    V.row(2) << 1;
    V.row(3) << 1;
    auto tag_handle = wmtk::mesh_utils::set_matrix_attribute(V, "tag", wmtk::PrimitiveType::Face, tm);
    
    // NOTE: I don't know why the following line gives me a linking error of undefined reference...
    // wmtk::TriMesh new_tm = wmtk::components::extract_subset(tm, tag_handle, 2);
    wmtk::TriMesh new_tm = wmtk::components::internal::extract_subset_2d(tm, tag_handle);
}