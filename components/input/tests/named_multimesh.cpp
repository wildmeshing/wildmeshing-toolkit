#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/input/input.hpp>
#include "tools/TriMesh_examples.hpp"
#include "wmtk/components/input/NamedMultiMesh.hpp"

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

using json = nlohmann::json;

namespace {
const std::filesystem::path data_dir = WMTK_DATA_DIR;

auto make_mesh()
{
    return wmtk::tests::disk(5);
}

auto make_child(wmtk::Mesh& m, const std::vector<int64_t>& path)
{
    if (path.size() == 0) {
        // input root mesh already exists so nothing to be done
        return;
    }
    for (size_t j = 1; j < path.size(); ++j) {
        std::vector<int64_t> p(path.begin(), path.begin() + j - 1);
        auto& cur_mesh = m.get_multi_mesh_mesh(p);
        int64_t child_index = path[j];
        const auto child_meshes = cur_mesh.get_child_meshes();
        for (int64_t index = child_meshes.size(); index < child_index; ++index) {
            auto new_mesh = make_mesh();
            auto map = wmtk::multimesh::same_simplex_dimension_bijection(cur_mesh, *new_mesh);
        }
    }
}
} // namespace


TEST_CASE("named_multimesh_parse", "[components][input]")
{
    {
        auto m = make_mesh();
        wmtk::components::input::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);

        assert(m == named_mm.root().shared_from_this());
        assert(m == named_mm.get_mesh(".").shared_from_this());
        assert(m == named_mm.get_mesh("").shared_from_this());
    }


    nlohmann::json js = nlohmann::json::array({"child"});
}
