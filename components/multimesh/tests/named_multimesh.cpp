#include <fmt/ranges.h>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/multimesh/NamedMultiMesh.hpp>
#include <wmtk/components/multimesh/utils/AttributeDescription.hpp>
#include <wmtk/components/multimesh/utils/get_attribute.hpp>
#include <wmtk/utils/Logger.hpp>

#include "utils.hpp"

using json = nlohmann::json;


TEST_CASE("named_multimesh_parse", "[components][multimesh]")
{
    {
        auto m = make_mesh();
        wmtk::components::multimesh::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);
        CHECK(std::vector<int64_t>{} == named_mm.get_id(""));
        CHECK(std::vector<int64_t>{} == named_mm.get_id("0"));
        CHECK(m == named_mm.get_mesh("").shared_from_this());

        named_mm.set_name("roo");

        CHECK(std::vector<int64_t>{} == named_mm.get_id(""));
        CHECK(m == named_mm.get_mesh("").shared_from_this());
        CHECK(std::vector<int64_t>{} == named_mm.get_id("roo"));
        CHECK(m == named_mm.root().shared_from_this());
        CHECK(m == named_mm.get_mesh("roo").shared_from_this());
    }


    {
        auto m = make_mesh();
        make_child(*m, {0});


        wmtk::components::multimesh::NamedMultiMesh named_mm;
        named_mm.set_mesh(*m);

        //CHECK(std::vector<int64_t>{} == named_mm.get_id(""));
        //CHECK(std::vector<int64_t>{} == named_mm.get_id("0"));
        //CHECK(m == named_mm.get_mesh("").shared_from_this());

        CHECK(std::vector<int64_t>{0} == named_mm.get_id("0.0"));
        CHECK(std::vector<int64_t>{0} == named_mm.get_id(".0"));

        CHECK(
            m->get_multi_mesh_child_mesh({0}).shared_from_this() ==
            named_mm.get_mesh("0.0").shared_from_this());
        CHECK(
            m->get_multi_mesh_child_mesh({0}).shared_from_this() ==
            named_mm.get_mesh(".0").shared_from_this());
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


            CHECK(std::vector<int64_t>{} == named_mm.get_id(""));
            auto check = [&](const std::vector<int64_t>& name) {
                CHECK(name == named_mm.get_id(fmt::format(".{}", fmt::join(name, "."))));
            };
            check({0});
            check({1});
            check({0, 0});
            check({1, 0});
            check({1, 1});
            check({0, 0, 0});
        }
        {
            nlohmann::json js;
            js["roo"]["c"]["d"]["e"] = {};
            js["roo"]["child"] = nlohmann::json::array({"c1", "c2"});
            std::cout << js << std::endl;
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

