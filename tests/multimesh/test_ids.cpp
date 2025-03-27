
#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;


TEST_CASE("test_absolute_ids", "[multimesh][ids]")
{
    auto root = disk(10);
    auto c0 = disk(10);
    auto map = wmtk::multimesh::same_simplex_dimension_bijection(*root, *c0);

    root->register_child_mesh(c0, map);

    auto c1 = disk(10);
    root->register_child_mesh(c1, map);

    auto c2 = disk(10);
    root->register_child_mesh(c2, map);

    auto c00 = disk(10);
    c0->register_child_mesh(c00, map);

    auto c01 = disk(10);
    c0->register_child_mesh(c01, map);

    auto c10 = disk(10);
    c1->register_child_mesh(c10, map);
    auto c100 = disk(10);
    c10->register_child_mesh(c100, map);
    auto c101 = disk(10);
    c10->register_child_mesh(c101, map);
    auto c1010 = disk(10);
    c101->register_child_mesh(c1010, map);

    auto debug_mm = [](const Mesh& m) -> const DEBUG_MultiMeshManager& {
        return reinterpret_cast<const DEBUG_TriMesh&>(m).multi_mesh_manager();
    };

    // chekc absolute ids
    REQUIRE(root->absolute_multi_mesh_id() == std::vector<int64_t>{});
    REQUIRE(c0->absolute_multi_mesh_id() == std::vector<int64_t>{0});
    REQUIRE(c1->absolute_multi_mesh_id() == std::vector<int64_t>{1});
    REQUIRE(c2->absolute_multi_mesh_id() == std::vector<int64_t>{2});
    REQUIRE(c00->absolute_multi_mesh_id() == std::vector<int64_t>{0, 0});
    REQUIRE(c01->absolute_multi_mesh_id() == std::vector<int64_t>{0, 1});
    REQUIRE(c10->absolute_multi_mesh_id() == std::vector<int64_t>{1, 0});
    REQUIRE(c100->absolute_multi_mesh_id() == std::vector<int64_t>{1, 0, 0});
    REQUIRE(c101->absolute_multi_mesh_id() == std::vector<int64_t>{1, 0, 1});
    REQUIRE(c1010->absolute_multi_mesh_id() == std::vector<int64_t>{1, 0, 1, 0});


    CHECK(debug_mm(*root).relative_id(*root, *root) == std::vector<int64_t>{});
    CHECK(debug_mm(*c0).relative_id(*c0, *root) == std::vector<int64_t>{0});
    CHECK(debug_mm(*c1).relative_id(*c1, *root) == std::vector<int64_t>{1});
    CHECK(debug_mm(*c2).relative_id(*c2, *root) == std::vector<int64_t>{2});
    CHECK(debug_mm(*c00).relative_id(*c00, *root) == std::vector<int64_t>{0, 0});
    CHECK(debug_mm(*c01).relative_id(*c01, *root) == std::vector<int64_t>{0, 1});
    CHECK(debug_mm(*c10).relative_id(*c10, *root) == std::vector<int64_t>{1, 0});
    CHECK(debug_mm(*c100).relative_id(*c100, *root) == std::vector<int64_t>{1, 0, 0});
    CHECK(debug_mm(*c101).relative_id(*c101, *root) == std::vector<int64_t>{1, 0, 1});
    CHECK(debug_mm(*c1010).relative_id(*c1010, *root) == std::vector<int64_t>{1, 0, 1, 0});

    std::vector<std::shared_ptr<Mesh>> meshes{root, c1, c1, c2, c00, c01, c10, c100, c101, c1010};

    for (const auto& mptr : meshes) {
        const DEBUG_MultiMeshManager& mm = debug_mm(*mptr);
        // check that the relative id to the root is the same as absolute id
        CHECK(mm.relative_id(*mptr, *root) == mptr->absolute_multi_mesh_id());
        CHECK(mm.relative_id(*mptr, *mptr) == std::vector<int64_t>{});


        // check that the relative id to self is empty
        CHECK(mm.relative_id(*mptr, *mptr).empty());

        // a mesh is a child of itself
        CHECK(mm.is_child(*mptr, *mptr));

        // check that if child then returns a relative id
        for (const auto& cptr : mptr->get_all_child_meshes()) {
            const DEBUG_MultiMeshManager& cmm = debug_mm(*cptr);

            CHECK(cmm.is_child(*cptr, *mptr));
            CHECK(!mm.is_child(*mptr, *cptr));
        }
        // check two versions of is_child
        for (const auto& nptr : meshes) {
            const DEBUG_MultiMeshManager& nmm = debug_mm(*nptr);

            CHECK(
                nmm.is_child(*nptr, *mptr) == wmtk::multimesh::MultiMeshManager::is_child(
                                                  nptr->absolute_multi_mesh_id(),
                                                  mptr->absolute_multi_mesh_id()));
        }
    }

    CHECK(c1010->get_all_child_meshes().size() == 0);
    CHECK(c101->get_all_child_meshes().size() == 1);
    CHECK(c100->get_all_child_meshes().size() == 0);
    CHECK(c10->get_all_child_meshes().size() == 3);
    CHECK(c01->get_all_child_meshes().size() == 0);
    CHECK(c00->get_all_child_meshes().size() == 0);
    CHECK(c0->get_all_child_meshes().size() == 2);
    CHECK(c1->get_all_child_meshes().size() == 4);
    CHECK(c2->get_all_child_meshes().size() == 0);
    CHECK(root->get_all_child_meshes().size() == 9);
}
