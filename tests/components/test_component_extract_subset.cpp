#include <wmtk_components/extract_subset/internal/extract_subset_2d.hpp>
#include <wmtk_components/extract_subset/extract_subset.hpp>
#include <catch2/catch_test_macros.hpp>
#include "../tools/TriMesh_examples.hpp"
#include "wmtk/attribute/Attribute.hpp"
#include "wmtk/Primitive.hpp"
#include "wmtk/attribute/AttributeHandle.hpp"
#include "wmtk/Mesh.hpp"
#include "wmtk/attribute/AccessorBase.hpp"

TEST_CASE("3_neighbors_test_case", "[components][extract_subset][2D]")
{
    wmtk::TriMesh tm;
    tm = wmtk::tests::three_neighbors(); // start with 4 tri
    auto tag_handle =
        tm.register_attribute<long>("tag", wmtk::PrimitiveType::Face, 1, false, 0);
    auto tag_acc = tm.create_accessor(tag_handle);

    // {
    //     std::vector<long> tag(4);
    //     tag = {1, 0, 1, 1};
    //     tag_acc.set_attribute(tag);
    // }

    // NOTE: The line commented out gives me a linking error of undefined reference
    // But the line after it runs normally
    
    // wmtk::TriMesh new_tm = wmtk::components::extract_subset(tm, tag_handle, 2);
    wmtk::TriMesh new_tm = wmtk::components::internal::extract_subset_2d(tm, tag_handle);
}