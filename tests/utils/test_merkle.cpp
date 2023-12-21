#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <wmtk/utils/merkle_tree.hpp>
#include <wmtk/utils/merkle_tree_diff.hpp>
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("test_default_merkle", "[merkle]")
{
    {
        auto mesh = two_neighbors();

        auto js = utils::merkle_tree(mesh);
        js["name"] = "two_neighbors";
        std::cout << js.dump(2) << std::endl;
    }


    {
        auto mesh = tests_3d::six_cycle_tets();

        auto js = utils::merkle_tree(mesh);
        js["name"] = "six_cycle_tets";
        std::cout << js.dump(2) << std::endl;
    }


    {
        auto a = two_neighbors();
        auto b = tests_3d::six_cycle_tets();

        CHECK(!utils::merkle_tree_diff(a, a).has_value());
        CHECK(!utils::merkle_tree_diff(b, b).has_value());
        {
            auto js_opt = utils::merkle_tree_diff(a, b, true);
            REQUIRE(js_opt.has_value());
            // std::cout << js_opt->dump(2) << std::endl;
        }
        auto c = quad();
        CHECK(!utils::merkle_tree_diff(c, c).has_value());
        {
            auto js_opt = utils::merkle_tree_diff(a, c, true);
            REQUIRE(js_opt.has_value());
            // std::cout << js_opt->dump(2) << std::endl;
            //std::ofstream ofs("/tmp/test.js");
            // ofs << js_opt->dump(2);
        }
    }
}
