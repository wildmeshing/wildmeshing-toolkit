
#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/multimesh/utils/transport_tuple.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "tools/all_valid_local_tuples.hpp"
#include "wmtk/multimesh/utils/find_local_dart_action.hpp"

using namespace wmtk;
using namespace wmtk::tests;

TEST_CASE("transport_tuple", "[tuple][multimesh]")
{
    // when other meshes are available add them here
    for (PrimitiveType base_mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        autogen::SimplexDart base_sd(base_mesh_type);

        auto base_all_tuples = all_valid_local_tuples(base_mesh_type);
        for (PrimitiveType mesh_type :
             {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
            auto all_tuples = all_valid_local_tuples(mesh_type);
            autogen::SimplexDart sd(mesh_type);

            for (const auto& base_source : base_all_tuples) {
                for (const auto& base_target : base_all_tuples) {
                    const int8_t base_action = wmtk::multimesh::utils::find_local_dart_action(
                        base_sd,
                        base_source,
                        base_target);
                    /*
                    spdlog::warn(
                        "Base action: {}={}>{}",
                        wmtk::utils::TupleInspector::as_string(base_source),
                        base_action,
                        wmtk::utils::TupleInspector::as_string(base_target));
                        */


                    if (base_mesh_type >= mesh_type) {
                        bool same_face = true;
                        for (PrimitiveType pt : utils::primitive_range(mesh_type, base_mesh_type)) {
                            if (pt == base_mesh_type) {
                                continue;
                            }
                            // only' map between things where a map exists
                            if (wmtk::utils::TupleInspector::local_id(pt, base_source) !=
                                wmtk::utils::TupleInspector::local_id(pt, base_target)) {
                                same_face = false;
                                break;
                            }
                        }
                        if (!same_face) {
                            continue;
                        }
                        for (const auto& source : all_tuples) {
                            wmtk::Tuple sequence_res =
                                wmtk::multimesh::utils::internal::transport_tuple_sequence(
                                    base_source,
                                    base_target,
                                    base_mesh_type,
                                    source,
                                    mesh_type);
                            wmtk::Tuple dart_res =
                                wmtk::multimesh::utils::internal::transport_tuple_dart(
                                    base_source,
                                    base_target,
                                    base_mesh_type,
                                    source,
                                    mesh_type);
                            const int8_t seq_action =
                                wmtk::multimesh::utils::find_local_dart_action(
                                    sd,
                                    source,
                                    sequence_res);
                            const int8_t action = wmtk::multimesh::utils::find_local_dart_action(
                                sd,
                                source,
                                dart_res);
                            /*
                            spdlog::info(
                                "Sequence action: {}={}>{}",
                                wmtk::utils::TupleInspector::as_string(source),
                                seq_action,
                                wmtk::utils::TupleInspector::as_string(sequence_res));

                            spdlog::info(
                                "Dart action: {}={}>{}",
                                wmtk::utils::TupleInspector::as_string(source),
                                action,
                                wmtk::utils::TupleInspector::as_string(dart_res));
                            */

                            CHECK(seq_action == action);
                            CHECK(base_sd.convert(base_action, sd) == action);

                            CHECK(sequence_res == dart_res);

                            continue;
                        }
                    }
                }
            }
        }
    }
}
