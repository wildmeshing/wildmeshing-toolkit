#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/components/output/OutputOptions.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/output/parse_output.hpp>

using json = nlohmann::json;
using namespace wmtk::components::output;

TEST_CASE("component_parse_output", "[components][output]")
{
    {
        std::string path = "file.hdf5";
        OutputOptions hdf5_opts;
        hdf5_opts.path = path;
        hdf5_opts.type = ".hdf5";
        OutputOptions vtu_opts;
        vtu_opts.path = "file";
        vtu_opts.type = ".vtu";

        {
            auto opts = parse_output(path);
            REQUIRE(opts.size() == 1);
            const auto& [name, opt] = opts[0];
            CHECK(name == "");
            CHECK(opt == hdf5_opts);
        }

        nlohmann::json hdf5 = {{"path", path}, {"type", ".hdf5"}};
        {
            auto opts = parse_output(hdf5);
            REQUIRE(opts.size() == 1);
            const auto& [name, opt] = opts[0];
            CHECK(name == "");
            CHECK(opt == hdf5_opts);
        }
        nlohmann::json vtu = {{"path", "file"}, {"type", ".vtu"}};
        {
            auto opts = parse_output(vtu);
            REQUIRE(opts.size() == 1);
            const auto& [name, opt] = opts[0];
            CHECK(name == "");
            CHECK(opt == vtu_opts);
        }
        nlohmann::json basic;
        basic["mesh"] = hdf5;
        basic["mesh2"] = vtu;

        {
            auto opts = parse_output(basic);
            REQUIRE(opts.size() == 2);
            {
                const auto& [name, opt] = opts[0];
                CHECK(name == "mesh");
                CHECK(opt == hdf5_opts);
            }
            {
                const auto& [name, opt] = opts[1];
                CHECK(name == "mesh2");
                CHECK(opt == vtu_opts);
            }
        }

        nlohmann::json array = {hdf5, vtu};

        {
            auto opts = parse_output(array);
            REQUIRE(opts.size() == 2);
            {
                const auto& [name, opt] = opts[0];
                CHECK(name == "");
                CHECK(opt == hdf5_opts);
            }
            {
                const auto& [name, opt] = opts[1];
                CHECK(name == "");
                CHECK(opt == vtu_opts);
            }
        }

        nlohmann::json array_with_names;
        nlohmann::json basic_hdf5;
        basic_hdf5["mesh"] = hdf5;
        nlohmann::json basic_vtu;
        basic_vtu["mesh2"] = vtu;
        array_with_names.push_back(basic_hdf5);
        array_with_names.push_back(basic_vtu);
        {
            auto opts = parse_output(array_with_names);
            REQUIRE(opts.size() == 2);
            {
                const auto& [name, opt] = opts[0];
                CHECK(name == "mesh");
                CHECK(opt == hdf5_opts);
            }
            {
                const auto& [name, opt] = opts[1];
                CHECK(name == "mesh2");
                CHECK(opt == vtu_opts);
            }
        }


        nlohmann::json array_both_same_mesh;
        array_both_same_mesh["mesh"] = {hdf5, vtu};
        {
            auto opts = parse_output(array_both_same_mesh);
            REQUIRE(opts.size() == 2);
            {
                const auto& [name, opt] = opts[0];
                CHECK(name == "mesh");
                CHECK(opt == hdf5_opts);
            }
            {
                const auto& [name, opt] = opts[1];
                CHECK(name == "mesh");
                CHECK(opt == vtu_opts);
            }
        }
    }
}
