
#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/input/InputOptions.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/input/mesh_with_tag_from_image.hpp>
#include <wmtk/components/mesh_info/simplex/volumes.hpp>
#include <wmtk/io/Cache.hpp>
#include "tools/TriMesh_examples.hpp"

using json = nlohmann::json;

namespace {
const std::filesystem::path data_dir = WMTK_DATA_DIR;
}

TEST_CASE("component_mesh_info_volumes_box", "[components][mesh_info][volumes]")
{
    const std::filesystem::path input_file = data_dir / "upsample_box.msh";
    std::shared_ptr<wmtk::Mesh> mesh_ptr = wmtk::components::input::input(input_file);

    auto trimesh_ptr = std::dynamic_pointer_cast<wmtk::TriMesh>(mesh_ptr);

    auto position =
        trimesh_ptr->get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);

    auto areas_transfer = wmtk::components::mesh_info::simplex::volumes<double>(
        position,
        wmtk::PrimitiveType::Triangle,
        "areas");
    auto length_transfer = wmtk::components::mesh_info::simplex::volumes<double>(
        position,
        wmtk::PrimitiveType::Edge,
        "length");

    auto areas_handle = areas_transfer->handle();
    auto length_handle = length_transfer->handle();

    auto areas_acc = trimesh_ptr->create_const_accessor<double, 1>(areas_handle);
    auto length_acc = trimesh_ptr->create_const_accessor<double, 1>(length_handle);

    // 4 different types of length
    for (auto e : trimesh_ptr->get_all(wmtk::PrimitiveType::Edge)) {
        double len = length_acc.const_scalar_attribute(e);
        if (len < 1.7e-2) {
            CHECK_THAT(len, Catch::Matchers::WithinRel(0.015625, 1e-5));
        } else if (len < 2.5e-2) {
            CHECK_THAT(len, Catch::Matchers::WithinRel(0.0220970869, 1e-5));
        } else if (len < 3.3e-2) {
            CHECK_THAT(len, Catch::Matchers::WithinRel(0.03125, 1e-5));
        } else {
            CHECK_THAT(len, Catch::Matchers::WithinRel(0.0349385621, 1e-3));
        }
    }


    // 2 different areas
    for (auto e : trimesh_ptr->get_all(wmtk::PrimitiveType::Triangle)) {
        double area = areas_acc.const_scalar_attribute(e);

        // edges can be either X or 2X in areagth
        if (area > 1.5e-4) {
            CHECK_THAT(area, Catch::Matchers::WithinRel(0.000244140625, 1e-5));
        } else {
            CHECK_THAT(area, Catch::Matchers::WithinRel(0.0001220703, 1e-5));
        }
    }
}
TEST_CASE("component_mesh_info_volumes_single_triangle", "[components][mesh_info][volumes]")
{
    auto trimesh = wmtk::tests::single_equilateral_triangle(2);


    auto position = trimesh.get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);

    auto areas_transfer = wmtk::components::mesh_info::simplex::volumes<double>(
        position,
        wmtk::PrimitiveType::Triangle,
        "areas");
    auto length_transfer = wmtk::components::mesh_info::simplex::volumes<double>(
        position,
        wmtk::PrimitiveType::Edge,
        "length");

    auto areas_handle = areas_transfer->handle();
    auto length_handle = length_transfer->handle();

    auto areas_acc = trimesh.create_const_accessor<double, 1>(areas_handle);
    auto length_acc = trimesh.create_const_accessor<double, 1>(length_handle);

    // 4 different types of length
    for (auto e : trimesh.get_all(wmtk::PrimitiveType::Edge)) {
        double len = length_acc.const_scalar_attribute(e);
        CHECK_THAT(len, Catch::Matchers::WithinRel(1.0, 1e-5));
    }


    // 2 different areas
    for (auto e : trimesh.get_all(wmtk::PrimitiveType::Triangle)) {
        double area = areas_acc.const_scalar_attribute(e);
        CHECK_THAT(area, Catch::Matchers::WithinRel(std::sqrt(3.0) / 4, 1e-5));
    }
}
