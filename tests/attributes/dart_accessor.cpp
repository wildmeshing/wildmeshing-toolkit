#include <numeric>

#include <spdlog/stopwatch.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/DartAccessor.hpp>
#include <wmtk/attribute/TupleAccessor.hpp>
#include <wmtk/io/read_mesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "tools/TriMesh_examples.hpp"

TEST_CASE("dart_access", "[dart_accessor]")

{
    auto mesh = wmtk::tests::three_neighbors();

    auto handle = wmtk::attribute::register_dart_attribute(mesh, "dart", true);

    wmtk::attribute::DartAccessor acc(mesh, handle);

    auto sd = wmtk::autogen::SimplexDart::get_singleton(wmtk::PrimitiveType::Triangle);


    for (const wmtk::Tuple& t : mesh.get_all(wmtk::PrimitiveType::Edge)) {
        wmtk::autogen::Dart d = sd.dart_from_tuple(t);
        for (wmtk::PrimitiveType pt : {wmtk::PrimitiveType::Vertex, wmtk::PrimitiveType::Edge}) {
            wmtk::Tuple ot = mesh.switch_tuple(t, pt);
            auto od = acc.switch_dart(d, pt);
            wmtk::autogen::Dart od2 = sd.dart_from_tuple(ot);
            CHECK(od.global_id() == od2.global_id());
            CHECK(od.local_orientation() == od2.local_orientation());
        }
        bool is_boundary_m = mesh.is_boundary(wmtk::PrimitiveType::Edge, t);
        bool is_boundary_d = acc.is_boundary(d);
        REQUIRE(is_boundary_m == is_boundary_d);
        if (!is_boundary_m) {
            wmtk::Tuple ot = mesh.switch_tuple(t, wmtk::PrimitiveType::Triangle);
            auto od = acc.switch_dart(d, wmtk::PrimitiveType::Triangle);
            wmtk::autogen::Dart od2 = sd.dart_from_tuple(ot);
            CHECK(od.global_id() == od2.global_id());
            CHECK(od.local_orientation() == od2.local_orientation());
        }
    }
}

const std::filesystem::path data_dir = WMTK_DATA_DIR;
TEST_CASE("dart_performance", "[performance][.]")
{
    const std::filesystem::path meshfile = data_dir / "armadillo.msh";
#if defined(_NDEBUG)
    const int64_t iterations = 1000;
#else
    const int64_t iterations = 100;
#endif

    auto mesh_in = std::static_pointer_cast<wmtk::TriMesh>(wmtk::io::read_mesh(meshfile));
    wmtk::TriMesh& mesh = *mesh_in;
    auto handle = wmtk::attribute::register_dart_attribute(mesh, "dart", true);
    std::vector<wmtk::Tuple> all_tuples = mesh.get_all(wmtk::PrimitiveType::Edge);
    auto sd = wmtk::autogen::SimplexDart::get_singleton(wmtk::PrimitiveType::Triangle);
    std::vector<wmtk::autogen::Dart> all_darts;
    for (const auto& t : all_tuples) {
        all_darts.emplace_back(sd.dart_from_tuple(t));
    }
    {
        spdlog::stopwatch sw;
        auto test_handle =
            mesh.register_attribute<int64_t>("Test attr", wmtk::PrimitiveType::Edge, 1, true);
        wmtk::attribute::DartAccessor acc(mesh, handle);

        auto test_acc = mesh.create_accessor<int64_t, 1>(test_handle);
        for (int j = 0; j < iterations; ++j) {
            for (const wmtk::autogen::Dart& d : all_darts) {
                bool is_boundary_d = acc.is_boundary(d);
                if (!is_boundary_d) {
                    auto od = acc.switch_dart(d, wmtk::PrimitiveType::Triangle);
                    test_acc.scalar_attribute(od) = od.global_id();
                }
            }
        }
        spdlog::info("Dart Elapsed: {} seconds", sw);
    }
    {
        spdlog::stopwatch sw;
        auto test_handle =
            mesh.register_attribute<int64_t>("Test attr", wmtk::PrimitiveType::Edge, 1, true);
        wmtk::attribute::DartAccessor acc(mesh, handle);

        auto test_acc = mesh.create_accessor<int64_t, 1>(test_handle);
        for (int j = 0; j < iterations; ++j) {
            for (const wmtk::Tuple& t : all_tuples) {
                wmtk::autogen::Dart d = sd.dart_from_tuple(t);

                bool is_boundary_d = acc.is_boundary(d);
                if (!is_boundary_d) {
                    auto od = acc.switch_dart(d, wmtk::PrimitiveType::Triangle);
                    wmtk::Tuple ot  = sd.tuple_from_dart(od);
                    test_acc.scalar_attribute(ot) = od.global_id();
                }
            }
        }
        spdlog::info("Dart from Tuple Elapsed: {} seconds", sw);
    }

    {
        spdlog::stopwatch sw;
        auto test_handle =
            mesh.register_attribute<int64_t>("Test attr", wmtk::PrimitiveType::Edge, 1, true);

        auto test_acc = mesh.create_accessor<int64_t, 1>(test_handle);

        for (int j = 0; j < iterations; ++j) {
            for (const wmtk::Tuple& t : all_tuples) {
                bool is_boundary_m = mesh.is_boundary(wmtk::PrimitiveType::Edge, t);
                if (!is_boundary_m) {
                    wmtk::Tuple ot = mesh.switch_tuple(t, wmtk::PrimitiveType::Triangle);
                    test_acc.scalar_attribute(ot) = wmtk::utils::TupleInspector::global_cid(ot);
                }
            }
        }
        spdlog::info("Tuple Elapsed: {} seconds", sw);
    }
}
