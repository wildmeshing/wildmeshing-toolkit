#include <fmt/ranges.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/NamedMultiMesh.hpp>
#include <wmtk/components/multimesh/utils/AttributeDescription.hpp>
#include <wmtk/components/multimesh/utils/get_attribute.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/components/multimesh/MeshCollection.hpp"

#include "utils.hpp"

using json = nlohmann::json;

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
        auto ad2 = js.get<AD>();
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

    {
        AD ad{"double_test", {}, AT::Double};
        JS js{{"path", "double_test"},  {"type", "double"}};
        check(ad, js);
    }
    {
        AD ad{"double_test", 2, {}};
        JS js{{"path", "double_test"}, {"dimension", 2}};
        check(ad, js);
    }
    {
        AD ad{"double_test", {}, {}};
        JS js{{"path", "double_test"}};
        check(ad, js);
    }
}
