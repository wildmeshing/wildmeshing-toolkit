
#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/input/mesh_with_tag_from_image.hpp>
#include <wmtk/components/mesh_info/simplex/dihedral_angles.hpp>

using json = nlohmann::json;

namespace {
const std::filesystem::path data_dir = WMTK_DATA_DIR;
}

TEST_CASE("component_mesh_info_dihedral_angles_box", "[components][mesh_info][dihedral_angles]")
{
    const std::filesystem::path input_file = data_dir / "upsample_box.msh";
    std::shared_ptr<wmtk::Mesh> mesh_ptr = wmtk::components::input::input(input_file);

    auto trimesh_ptr = std::dynamic_pointer_cast<wmtk::TriMesh>(mesh_ptr);

    auto position =
        trimesh_ptr->get_attribute_handle<double>("vertices", wmtk::PrimitiveType::Vertex);

    auto angles_transfer =
        wmtk::components::mesh_info::simplex::dihedral_angles<double>(position, "angles");
    auto angles_handle = angles_transfer->handle();

    auto angles_acc = trimesh_ptr->create_const_accessor<double, 1>(angles_handle);

    for (auto e : trimesh_ptr->get_all(wmtk::PrimitiveType::Edge)) {
        double ang = angles_acc.const_scalar_attribute(e);
        // spdlog::info("{}", ang);
        if (ang > 2) {
            CHECK_THAT(ang, Catch::Matchers::WithinRel(M_PI, 1e-5));
        } else {
            CHECK_THAT(ang, Catch::Matchers::WithinRel(M_PI / 2, 1e-5));
        }

        // wmtk::components::mesh_info::simplex::dihedral_angles(
    }
}
