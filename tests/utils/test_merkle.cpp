#include <catch2/catch_test_macros.hpp>
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"
#include "../tools/TetMesh_examples.hpp"
#include <wmtk/utils/merkle_tree.hpp>

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
}
