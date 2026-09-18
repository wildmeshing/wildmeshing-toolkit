#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/PolyfemJson.hpp>

using wmtk::components::polyfem_ops::deep_merge;
using wmtk::components::polyfem_ops::OrderedJson;
using wmtk::components::polyfem_ops::resolve_amips_weights;

TEST_CASE("polyfem_ops deep_merge", "[components][polyfem_ops]")
{
    // deep_merge is what lets a caller override one paraview option without restating the rest,
    // and what `geometry_block` applies to the transformation.
    SECTION("the override wins on a key both carry")
    {
        const OrderedJson base = {{"a", 1}, {"b", 2}};
        const OrderedJson over = {{"b", 3}};
        REQUIRE(deep_merge(base, over) == OrderedJson({{"a", 1}, {"b", 3}}));
    }
    SECTION("nested objects merge instead of replacing each other")
    {
        const OrderedJson base = {{"options", {{"material", true}, {"nodes", false}}}};
        const OrderedJson over = {{"options", {{"nodes", true}}}};
        const OrderedJson merged = deep_merge(base, over);
        REQUIRE(merged["options"]["material"] == true);
        REQUIRE(merged["options"]["nodes"] == true);
    }
    SECTION("a non-object override replaces an object")
    {
        const OrderedJson base = {{"options", {{"material", true}}}};
        const OrderedJson over = {{"options", 5}};
        REQUIRE(deep_merge(base, over)["options"] == 5);
    }
    SECTION("keys the override adds are appended, keeping the base's order first")
    {
        const OrderedJson base = {{"a", 1}};
        const OrderedJson over = {{"b", 2}};
        const OrderedJson merged = deep_merge(base, over);
        REQUIRE(merged.dump() == "{\"a\":1,\"b\":2}");
    }
    SECTION("an empty override is a copy -- the call build_polyfem_json actually makes")
    {
        const OrderedJson base = {{"a", {{"b", 1}}}};
        REQUIRE(deep_merge(base, OrderedJson::object()) == base);
    }
}

TEST_CASE("polyfem_ops resolves AMIPS weights", "[components][polyfem_ops]")
{
    // The reduced mesh has two materials, so whatever the caller spelled has to come out as
    // exactly {"ambient": w, "body": w}.
    SECTION("nothing given: the engine defaults, 1e-6 ambient and 1 body")
    {
        const OrderedJson w = resolve_amips_weights(OrderedJson::object());
        REQUIRE(w["ambient"] == 1e-6);
        REQUIRE(w["body"] == 1e0);
    }
    SECTION("the legacy dict: its 'ambient' entry, and its first non-ambient value for the body")
    {
        OrderedJson cfg;
        cfg["amips_weights"] = {{"ambient", 2.0}, {"tag_0", 3.0}};
        const OrderedJson w = resolve_amips_weights(cfg);
        REQUIRE(w["ambient"] == 2.0);
        REQUIRE(w["body"] == 3.0);
    }
    SECTION("a non-ambient entry alone leaves the ambient default in place")
    {
        OrderedJson cfg;
        cfg["amips_weights"] = {{"tag_0", 3.0}};
        const OrderedJson w = resolve_amips_weights(cfg);
        REQUIRE(w["ambient"] == 1e-6);
        REQUIRE(w["body"] == 3.0);
    }
    SECTION("the dedicated keys win over the legacy dict")
    {
        OrderedJson cfg;
        cfg["amips_weights"] = {{"ambient", 2.0}, {"tag_0", 3.0}};
        cfg["amips_ambient_weight"] = 7.0;
        cfg["amips_body_weight"] = 8.0;
        const OrderedJson w = resolve_amips_weights(cfg);
        REQUIRE(w["ambient"] == 7.0);
        REQUIRE(w["body"] == 8.0);
    }
    SECTION("the smoothing engine's body default reaches the body weight")
    {
        // laplacian_smoothing.run sets amips_body_weight to 1e-4 and nothing else; the ambient
        // weight must still come from the engine default.
        OrderedJson cfg;
        cfg["amips_body_weight"] = 1e-4;
        const OrderedJson w = resolve_amips_weights(cfg);
        REQUIRE(w["ambient"] == 1e-6);
        REQUIRE(w["body"] == 1e-4);
    }
}
