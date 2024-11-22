#include <fmt/ranges.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/NamedMultiMesh.hpp>
#include <wmtk/components/multimesh/utils/AttributeDescription.hpp>
#include <wmtk/components/multimesh/utils/get_attribute.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/TriMesh_examples.hpp"
#include "wmtk/components/multimesh/MeshCollection.hpp"

#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>

using json = nlohmann::json;

namespace {
const std::filesystem::path data_dir = WMTK_DATA_DIR;

auto make_mesh()
{
    return wmtk::tests::disk(5);
}

auto make_child(wmtk::Mesh& m, const std::vector<int64_t>& path)
    -> std::vector<std::shared_ptr<wmtk::Mesh>>
{
    if (path.size() == 0) {
        // multimesh root mesh already exists so nothing to be done
        return {};
    }
    std::vector<std::shared_ptr<wmtk::Mesh>> meshes;
    for (size_t j = 0; j < path.size(); ++j) {
        std::vector<int64_t> p(path.begin(), path.begin() + j);
        auto& cur_mesh = m.get_multi_mesh_mesh(p);
        int64_t child_index = path[j];
        const auto child_meshes = cur_mesh.get_child_meshes();
        for (int64_t index = child_meshes.size(); index <= child_index; ++index) {
            auto new_mesh = make_mesh();
            auto map = wmtk::multimesh::same_simplex_dimension_bijection(cur_mesh, *new_mesh);

            cur_mesh.register_child_mesh(new_mesh, map);
            meshes.emplace_back(new_mesh);
        }
    }
    return meshes;
}
} // namespace


TEST_CASE("named_multimesh_parse", "[components][multimesh]")
{
    {
        auto m = make_mesh();
        wmtk::components::multimesh::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);

        named_mm.set_name("roo");

        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(m == named_mm.root().shared_from_this());
        CHECK(m == named_mm.get_mesh("roo").shared_from_this());
    }


    {
        auto m = make_mesh();
        make_child(*m, {0});


        wmtk::components::multimesh::NamedMultiMesh named_mm;
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
        wmtk::components::multimesh::NamedMultiMesh named_mm;
        nlohmann::json js;
        js["roo"] = nlohmann::json("child");
        named_mm.set_names(js);
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
    }
    {
        wmtk::components::multimesh::NamedMultiMesh named_mm;
        nlohmann::json js;
        js["roo"]["child"] = {};
        named_mm.set_names(js);
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
    }

    {
        wmtk::components::multimesh::NamedMultiMesh named_mm;
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

TEST_CASE("named_multimesh_parse_attributes", "[components][multimesh]")
{
    {
        auto m = make_mesh();
        wmtk::components::multimesh::MeshCollection mc;
        wmtk::components::multimesh::NamedMultiMesh& named_mm =
            mc.emplace_mesh(*m, std::string("roo"));

        auto double_test_handle =
            m->register_attribute<double>("double_test", wmtk::PrimitiveType::Vertex, 1);
        auto char_test_handle =
            m->register_attribute<char>("char_test", wmtk::PrimitiveType::Vertex, 1);
        auto int_test_handle =
            m->register_attribute<int64_t>("int_test", wmtk::PrimitiveType::Vertex, 1);
        auto rational_test_handle =
            m->register_attribute<wmtk::Rational>("rational_test", wmtk::PrimitiveType::Vertex, 1);

        using AT = wmtk::attribute::AttributeType;

using AD = wmtk::components::multimesh::utils::AttributeDescription;
        { // double check that path extraction is working
            std::vector<AD> double_ads;
            double_ads.emplace_back(AD{"double_test", 0, AT::Double});
            double_ads.emplace_back(AD{"/double_test", 0, AT::Double});
            double_ads.emplace_back(AD{"roo/double_test", 0, AT::Double});


            for (const auto& ad : double_ads) {
                auto h = wmtk::components::multimesh::utils::get_attribute(named_mm, ad);
                CHECK(double_test_handle == h);
                // just on mesh also works
                auto h2 = wmtk::components::multimesh::utils::get_attribute(*m, ad);
                CHECK(double_test_handle == h2);
                //// meshcollection too
                auto h3 = wmtk::components::multimesh::utils::get_attribute(mc, ad);
                CHECK(double_test_handle == h3);
            }
        }
        { // check that other types work
            CHECK(
                rational_test_handle == wmtk::components::multimesh::utils::get_attribute(
                                            named_mm,
                                            AD{"rational_test", 0, AT::Rational}));
            CHECK(
                int_test_handle == wmtk::components::multimesh::utils::get_attribute(
                                       named_mm,
                                       AD{"int_test", 0, AT::Int64}));
            CHECK(
                char_test_handle == wmtk::components::multimesh::utils::get_attribute(
                                        named_mm,
                                        AD{"char_test", 0, AT::Char}));
        }
        { // check that other simplex types work
            auto edge_handle =
                m->register_attribute<double>("double_test", wmtk::PrimitiveType::Edge, 1);
            auto tri_handle =
                m->register_attribute<double>("double_test", wmtk::PrimitiveType::Triangle, 1);
            CHECK(
                edge_handle == wmtk::components::multimesh::utils::get_attribute(
                                   named_mm,
                                   AD{"double_test", 1, AT::Double}));
            CHECK(
                tri_handle == wmtk::components::multimesh::utils::get_attribute(
                                  named_mm,
                                  AD{"double_test", 2, AT::Double}));
            // TODO: lazy about testing tet
        }
    }


    {
    using AD = wmtk::components::multimesh::utils::AttributeDescription;
        auto m = make_mesh();
        auto children = make_child(*m, {0});
        REQUIRE(children.size() == 1);
        auto child = children[0];


        wmtk::components::multimesh::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);
        {
            nlohmann::json js;
            js["roo"] = nlohmann::json::array({"child"});
            named_mm.set_names(js);
        }
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id("roo.child"));
        auto attr_handle =
            m->register_attribute<double>("double_test", wmtk::PrimitiveType::Vertex, 1);
        auto child_attr_handle =
            child->register_attribute<double>("double_test", wmtk::PrimitiveType::Vertex, 1);
        using AT = wmtk::attribute::AttributeType;
        CHECK(
            attr_handle == wmtk::components::multimesh::utils::get_attribute(
                               named_mm,
                               AD{"double_test", 0, AT::Double}));
        CHECK(
            attr_handle == wmtk::components::multimesh::utils::get_attribute(
                               named_mm,
                               AD{"/double_test", 0, AT::Double}));
        CHECK(
            attr_handle == wmtk::components::multimesh::utils::get_attribute(
                               named_mm,
                               AD{"roo/double_test", 0, AT::Double}));
        CHECK(
            child_attr_handle == wmtk::components::multimesh::utils::get_attribute(
                                     named_mm,
                                     AD{"roo.child/double_test", 0, AT::Double}));
        CHECK(
            child_attr_handle == wmtk::components::multimesh::utils::get_attribute(
                                     named_mm,
                                     AD{".child/double_test", 0, AT::Double}));
    }
}

TEST_CASE("multimesh_attribute_description_json", "[components][multimesh]")
{
    using AT = wmtk::attribute::AttributeType;
    using AD = wmtk::components::multimesh::utils::AttributeDescription;
    using JS = nlohmann::json;
    auto check = [](const AD& ad, const JS& js) {
        JS js2 = ad;
        CHECK(js2 == js);
        CHECK(ad == js.get<AD>());
    };
    {
        AD ad{"double_test", 0, AT::Double};
        JS js{{"path", "double_test"}, {"dimension", 0}, {"type", "double"}};
        check(ad, js);
    }
    {
        AD ad{"rational_test", 0, AT::Rational};
        JS js{{"path", "rational_test"}, {"dimension", 0}, {"type", "rational"}};
        check(ad, js);
    }
    {
        AD ad{"int_test", 0, AT::Int64};
        JS js{{"path", "int_test"}, {"dimension", 0}, {"type", "int"}};
        check(ad, js);
    }
    {
        AD ad{"char_test", 0, AT::Char};
        JS js{{"path", "char_test"}, {"dimension", 0}, {"type", "char"}};
        check(ad, js);
    }
    {
        AD ad{"double_test", 1, AT::Double};
        JS js{{"path", "double_test"}, {"dimension", 1}, {"type", "double"}};
        check(ad, js);
    }
    {
        AD ad{"double_test", 2, AT::Double};
        JS js{{"path", "double_test"}, {"dimension", 2}, {"type", "double"}};
        check(ad, js);
    }
}
