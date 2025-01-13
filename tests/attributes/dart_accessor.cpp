#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/DartAccessor.hpp>
#include <wmtk/attribute/TupleAccessor.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "tools/TriMesh_examples.hpp"

TEST_CASE("dart_access", "[dart_accessor]")

{
    auto mesh = wmtk::tests::three_neighbors();
    spdlog::info("Hi");

    auto handle = wmtk::attribute::register_dart_attribute(mesh, "dart", true);

    wmtk::attribute::DartAccessor acc(mesh, handle);

    auto sd = wmtk::autogen::SimplexDart::get_singleton(wmtk::PrimitiveType::Triangle);


    for (const wmtk::Tuple& t : mesh.get_all(wmtk::PrimitiveType::Edge)) {
        wmtk::autogen::Dart d = sd.dart_from_tuple(t);
        for (wmtk::PrimitiveType pt : {wmtk::PrimitiveType::Vertex, wmtk::PrimitiveType::Edge}) {
            wmtk::Tuple ot = mesh.switch_tuple(t, pt);
            auto od = acc.switch_dart(d, pt);
            wmtk::autogen::Dart od2 = sd.dart_from_tuple(ot);
            CHECK(od.as_tuple() == od2.as_tuple());
        }
        bool is_boundary_m = mesh.is_boundary(wmtk::PrimitiveType::Edge, t);
        bool is_boundary_d = acc.is_boundary(d);
        REQUIRE(is_boundary_m == is_boundary_d);
        if(is_boundary_m) {
            wmtk::Tuple ot = mesh.switch_tuple(t, wmtk::PrimitiveType::Triangle);
            auto od = acc.switch_dart(d, wmtk::PrimitiveType::Triangle);
            wmtk::autogen::Dart od2 = sd.dart_from_tuple(ot);
            CHECK(od.as_tuple() == od2.as_tuple());
        }
    }
}
