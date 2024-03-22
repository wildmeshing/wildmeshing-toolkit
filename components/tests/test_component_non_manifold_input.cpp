#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/base/Paths.hpp>
#include <wmtk/components/non_manifold_input/non_manifold_input.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

using namespace wmtk::components::base;
using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_non_manifold_input", "[components][non_manifold_input]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");

    json j;

    size_t nmv_expect = 0;
    size_t nme_expect = 0;

    SECTION("manifold")
    {
        const std::filesystem::path input_file = data_dir / "200683_sf.msh";
        j = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()},
            {"non_manifold_vertex_label", "nmv"},
            {"non_manifold_edge_label", "nme"},
            {"non_manifold_tag_value", 1}};
    }
    SECTION("non_manifold_edge")
    {
        const std::filesystem::path input_file = data_dir / "edge_with_four_triangles.msh";
        j = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()},
            {"non_manifold_vertex_label", "nmv"},
            {"non_manifold_edge_label", "nme"},
            {"non_manifold_tag_value", 1}};

        nmv_expect = 2;
        nme_expect = 1;
    }
    SECTION("non_manifold_vertex")
    {
        const std::filesystem::path input_file = data_dir / "hour_glass.msh";
        j = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", input_file.string()},
            {"ignore_z", false},
            {"tetrahedron_attributes", json::array()},
            {"non_manifold_vertex_label", "nmv"},
            {"non_manifold_edge_label", "nme"},
            {"non_manifold_tag_value", 1}};

        nmv_expect = 1;
        nme_expect = 0;
    }

    Paths p;

    CHECK_NOTHROW(wmtk::components::non_manifold_input(p, j, cache));

    // validation
    {
        auto m = cache.read_mesh("input_mesh");

        REQUIRE(m->has_attribute<int64_t>("nmv", PrimitiveType::Vertex));
        REQUIRE(m->has_attribute<int64_t>("nme", PrimitiveType::Edge));

        auto nmv_handle = m->get_attribute_handle<int64_t>("nmv", PrimitiveType::Vertex);
        auto nme_handle = m->get_attribute_handle<int64_t>("nme", PrimitiveType::Edge);
        auto nmv_acc = m->create_accessor<int64_t>(nmv_handle);
        auto nme_acc = m->create_accessor<int64_t>(nme_handle);

        size_t nmv_counter = 0;
        size_t nme_counter = 0;

        for (const Tuple& t : m->get_all(PrimitiveType::Vertex)) {
            if (nmv_acc.const_scalar_attribute(t) == 1) {
                ++nmv_counter;
            }
        }
        for (const Tuple& t : m->get_all(PrimitiveType::Edge)) {
            if (nme_acc.const_scalar_attribute(t) == 1) {
                ++nme_counter;
            }
        }

        CHECK(nmv_counter == nmv_expect);
        CHECK(nme_counter == nme_expect);


        if (true) {
            wmtk::io::ParaviewWriter
                writer("mesh_with_non_manifold_tags", "vertices", *m, true, true, true, false);
            m->serialize(writer);
        }
    }
}