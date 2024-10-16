#include <fmt/ranges.h>
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
    for (size_t j = 0; j < path.size(); ++j) {
        std::vector<int64_t> p(path.begin(), path.begin() + j);
        auto& cur_mesh = m.get_multi_mesh_mesh(p);
        int64_t child_index = path[j];
        const auto child_meshes = cur_mesh.get_child_meshes();
        for (int64_t index = child_meshes.size(); index <= child_index; ++index) {
            auto new_mesh = make_mesh();
            auto map = wmtk::multimesh::same_simplex_dimension_bijection(cur_mesh, *new_mesh);

            cur_mesh.register_child_mesh(new_mesh, map);
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

        named_mm.set_name("roo");

        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(m == named_mm.root().shared_from_this());
        CHECK(m == named_mm.get_mesh("roo").shared_from_this());
    }


    {
        auto m = make_mesh();
        make_child(*m, {0});


        wmtk::components::input::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);
        {
            nlohmann::json js;
            js["roo"] = nlohmann::json::array({"child"});
            named_mm.set_names(js);
        }
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
        CHECK(m == named_mm.root().shared_from_this());
        CHECK(m == named_mm.get_mesh("roo").shared_from_this());
        CHECK(
            m->get_multi_mesh_child_mesh({0}).shared_from_this() ==
            named_mm.get_mesh("roo.child").shared_from_this());
    }
    {
        wmtk::components::input::NamedMultiMesh named_mm;
        nlohmann::json js;
        js["roo"] = nlohmann::json("child");
        named_mm.set_names(js);
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
    }
    {
        wmtk::components::input::NamedMultiMesh named_mm;
        nlohmann::json js;
        js["roo"]["child"] = {};
        named_mm.set_names(js);
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
    }

    {
        wmtk::components::input::NamedMultiMesh named_mm;
        auto m = make_mesh();
        {
            make_child(*m, {0});
            make_child(*m, {0, 0, 0});
            make_child(*m, {1, 1});

            named_mm.set_mesh(*m);
        }
        {
            nlohmann::json js;
            js["roo"]["c"]["d"]["e"] = {};
            js["roo"]["child"] = nlohmann::json::array({"c1", "c2"});
            named_mm.set_names(js);
        }
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0, 0, 0} == named_mm.get_id("roo.c.d.e"));
        CHECK(std::vector<int64_t>{1} == named_mm.get_id("roo.child"));
        CHECK(std::vector<int64_t>{1, 0} == named_mm.get_id("roo.child.c1"));
        CHECK(std::vector<int64_t>{1, 1} == named_mm.get_id("roo.child.c2"));
        CHECK(
            m->get_multi_mesh_child_mesh({0, 0, 0}).shared_from_this() ==
            named_mm.get_mesh("roo.c.d.e").shared_from_this());
        CHECK(
            m->get_multi_mesh_child_mesh({1, 1}).shared_from_this() ==
            named_mm.get_mesh("roo.child.c2").shared_from_this());

        CHECK(
            m->get_multi_mesh_child_mesh({0, 0, 0}).shared_from_this() ==
            named_mm.get_mesh(".c.d.e").shared_from_this());
        CHECK(
            m->get_multi_mesh_child_mesh({1, 1}).shared_from_this() ==
            named_mm.get_mesh(".child.c2").shared_from_this());
    }
}
