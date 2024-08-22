#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/is_ccw.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/autogen/utils/subdart_maximal_action_to_face.hpp>
#include "tools/DEBUG_Tuple.hpp"
#include "tools/all_valid_local_tuples.hpp"
using namespace wmtk;
using namespace wmtk::autogen;
using namespace wmtk::tests;

TEST_CASE("tuple_autogen_index_dart_vs_switch", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        auto tuples = all_valid_local_tuples(mesh_type);
        autogen::SimplexDart sd(mesh_type);

        for (const auto& t : tuples) {
            CHECK(tuple_is_valid_for_ccw(mesh_type, t));
            int8_t tuple_as_index = sd.valid_index_from_tuple(t);
            for (PrimitiveType pt : primitives_up_to(mesh_type)) {
                Tuple manual_switch = local_switch_tuple(mesh_type, t, pt);

                const int8_t index_switch = sd.primitive_as_index(pt);

                Tuple table_switch = local_switch_tuple(mesh_type, t, index_switch);

                CHECK(manual_switch == table_switch);
            }
        }
    }
}

TEST_CASE("tuple_autogen_products_vs_switch", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        auto tuples = all_valid_local_tuples(mesh_type);

        std::vector<PrimitiveType> sequence;

        autogen::SimplexDart sd(mesh_type);


        for (const auto& t : tuples) {
            CHECK(tuple_is_valid_for_ccw(mesh_type, t));
            auto run = [&]() {
                Tuple manual_switch = t;
                const int8_t initial_index = sd.valid_index_from_tuple(t);
                const autogen::Dart initial_dart = sd.dart_from_tuple(t);
                int8_t index = initial_index;
                int8_t op = sd.identity();

                std::vector<int8_t> seq_tups;
                for (const auto& s : sequence) {
                    manual_switch = local_switch_tuple(mesh_type, manual_switch, s);
                    seq_tups.emplace_back(sd.primitive_as_index(s));
                    op = sd.product(sd.primitive_as_index(s), op);
                    index = sd.product(sd.primitive_as_index(s), index);
                }

                Tuple product_switch = sd.update_tuple_from_valid_index(t, index);
                Tuple product_switch2 =
                    sd.update_tuple_from_valid_index(t, sd.product(op, initial_index));

                const autogen::Dart output = sd.act(initial_dart, op);

                const autogen::Dart expected = sd.dart_from_tuple(product_switch);

                CHECK(manual_switch == product_switch);
                CHECK(product_switch == product_switch2);
                CHECK(output == expected);
            };
            for (size_t j = 0; j < 4; ++j) {
                for (PrimitiveType pt0 : primitives_up_to(mesh_type)) {
                    sequence.clear();
                    sequence.emplace_back(pt0);
                    if (j == 0) {
                        run();
                        continue;
                    }
                    for (PrimitiveType pt1 : primitives_up_to(mesh_type)) {
                        sequence.emplace_back(pt0);
                        if (j == 1) {
                            run();
                            continue;
                        }
                        for (PrimitiveType pt2 : primitives_up_to(mesh_type)) {
                            sequence.emplace_back(pt0);
                            if (j == 2) {
                                run();
                                continue;
                            }
                            for (PrimitiveType pt3 : primitives_up_to(mesh_type)) {
                                sequence.emplace_back(pt0);
                                if (j == 3) {
                                    run();
                                    continue;
                                }
                                for (PrimitiveType pt4 : primitives_up_to(mesh_type)) {
                                    sequence.emplace_back(pt0);
                                    if (j == 4) {
                                        run();
                                        continue;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("tuple_autogen_index_dart_map_between_simplices", "[tuple]")
{
    // Checks whether the mapping between simplices of different dimensions results in a
    // homomorphism
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        autogen::SimplexDart sd(mesh_type);
        for (PrimitiveType mesh_type2 :
             {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
            if (mesh_type > mesh_type2) {
                continue;
            }

            autogen::SimplexDart sd2(mesh_type2);
            for (int8_t index = 0; index < sd.size(); ++index) {
                // make sure mapping to a higher dimensional simplex and back always returns the
                // same dart (injectivity)
                REQUIRE(sd.convert(index, sd2) != -1);
                const int8_t index2 = sd.convert(index, sd2);
                CHECK(sd2.convert(index2, sd) == index);


                // mesh_type <= mesh_type2

                for (PrimitiveType sw = PrimitiveType::Vertex; sw < mesh_type; sw = sw + 1) {
                    int8_t sw_act2 = sd2.primitive_as_index(sw);
                    REQUIRE(sd2.product(sw_act2, sw_act2) == sd2.identity());
                    const int8_t index2_sw = sd2.product(index2, sw_act2);
                    int8_t sw_act = sd.primitive_as_index(sw);
                    const int8_t index_sw = sd.product(index, sw_act);
                    CHECK(sd2.convert(index2_sw, sd) == index_sw);
                    CHECK(sd.convert(index_sw, sd2) == index2_sw);
                }
            }

            // check for inversion part of homomorphism property
            for (int8_t index = 0; index < sd.size(); ++index) {
                const int8_t inv = sd.inverse(index);

                int8_t index2 = sd.convert(index, sd2);
                int8_t inv2 = sd.convert(inv, sd2);
                CHECK(sd2.product(index2, inv2) == sd2.identity());
            }
            // check for product part of homomorphism
            for (int8_t index = 0; index < sd.size(); ++index) {
                for (int8_t index_ = 0; index_ < sd.size(); ++index_) {
                    int8_t p = sd.product(index, index_);


                    int8_t index2 = sd.convert(index, sd2);
                    int8_t index_2 = sd.convert(index_, sd2);
                    int8_t p2 = sd2.product(index2, index_2);
                    CHECK(sd.convert(p, sd2) == p2);
                }
            }
        }
    }
}

namespace {
int8_t equal_subdart_dimension(PrimitiveType mesh_type, const Tuple& a, const Tuple& b)
{
    const auto& a_ = reinterpret_cast<const DEBUG_Tuple&>(a);
    const auto& b_ = reinterpret_cast<const DEBUG_Tuple&>(b);
    if (a_.local_vid() != b_.local_vid() || mesh_type == PrimitiveType::Vertex) {
        return 0;
    }
    if (a_.local_eid() != b_.local_eid() || mesh_type == PrimitiveType::Edge) {
        return 1;
    }
    if (a_.local_fid() != b_.local_fid() || mesh_type == PrimitiveType::Triangle) {
        return 2;
    }
    return 3;
}
int8_t equal_subdart_dimension(PrimitiveType mesh_type, int8_t a, int8_t b)
{
    const auto& sd = autogen::SimplexDart::get_singleton(mesh_type);

    return equal_subdart_dimension(
        mesh_type,
        sd.tuple_from_valid_index(0, a),
        sd.tuple_from_valid_index(0, b));
}

constexpr int fact(int n)
{
    return n <= 1 ? 1 : n * fact(n - 1);
}

constexpr int8_t simplex_count(PrimitiveType mesh_type, PrimitiveType type)
{
    int lower = int(type) + 1;
    int upper = int(mesh_type) + 1;
    // n! * r! / (n-r)!

    return fact(upper) / (fact(lower) * fact(upper - lower));
}
} // namespace

TEST_CASE("maximal_subdart_switches_checker_checker", "[tuple]")
{
    CHECK(
        equal_subdart_dimension(PrimitiveType::Vertex, Tuple(0, 0, 0, 0), Tuple(0, 0, 0, 0)) == 0);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Vertex, Tuple(0, 0, 0, 0), Tuple(1, 0, 0, 0)) == 0);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Vertex, Tuple(0, 0, 0, 0), Tuple(0, 1, 0, 0)) == 0);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Vertex, Tuple(0, 0, 0, 0), Tuple(0, 0, 1, 0)) == 0);
    CHECK(equal_subdart_dimension(PrimitiveType::Edge, Tuple(0, 0, 0, 0), Tuple(0, 0, 0, 0)) == 1);
    CHECK(equal_subdart_dimension(PrimitiveType::Edge, Tuple(0, 0, 0, 0), Tuple(1, 0, 0, 0)) == 0);
    CHECK(equal_subdart_dimension(PrimitiveType::Edge, Tuple(0, 0, 0, 0), Tuple(0, 1, 0, 0)) == 1);
    CHECK(equal_subdart_dimension(PrimitiveType::Edge, Tuple(0, 0, 0, 0), Tuple(0, 0, 1, 0)) == 1);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Triangle, Tuple(0, 0, 0, 0), Tuple(0, 0, 0, 0)) ==
        2);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Triangle, Tuple(0, 0, 0, 0), Tuple(1, 0, 0, 0)) ==
        0);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Triangle, Tuple(0, 0, 0, 0), Tuple(0, 1, 0, 0)) ==
        1);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Triangle, Tuple(0, 0, 0, 0), Tuple(0, 0, 1, 0)) ==
        2);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Tetrahedron, Tuple(0, 0, 0, 0), Tuple(0, 0, 0, 0)) ==
        3);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Tetrahedron, Tuple(0, 0, 0, 0), Tuple(1, 0, 0, 0)) ==
        0);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Tetrahedron, Tuple(0, 0, 0, 0), Tuple(0, 1, 0, 0)) ==
        1);
    CHECK(
        equal_subdart_dimension(PrimitiveType::Tetrahedron, Tuple(0, 0, 0, 0), Tuple(0, 0, 1, 0)) ==
        2);


    CHECK(simplex_count(PrimitiveType::Vertex, PrimitiveType::Vertex) == 1);

    CHECK(simplex_count(PrimitiveType::Edge, PrimitiveType::Vertex) == 2);
    CHECK(simplex_count(PrimitiveType::Edge, PrimitiveType::Edge) == 1);

    CHECK(simplex_count(PrimitiveType::Triangle, PrimitiveType::Vertex) == 3);
    CHECK(simplex_count(PrimitiveType::Triangle, PrimitiveType::Edge) == 3);
    CHECK(simplex_count(PrimitiveType::Triangle, PrimitiveType::Triangle) == 1);

    CHECK(simplex_count(PrimitiveType::Tetrahedron, PrimitiveType::Vertex) == 4);
    CHECK(simplex_count(PrimitiveType::Tetrahedron, PrimitiveType::Edge) == 6);
    CHECK(simplex_count(PrimitiveType::Tetrahedron, PrimitiveType::Triangle) == 4);
    CHECK(simplex_count(PrimitiveType::Tetrahedron, PrimitiveType::Tetrahedron) == 1);
}

TEST_CASE("maximal_subdart_switches", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        spdlog::info("Checking out {}", primitive_type_name(mesh_type));
        autogen::SimplexDart sd(mesh_type);
        for (int8_t index = 0; index < sd.size(); ++index) {
            for (PrimitiveType simplex_type :
                 {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
                if (mesh_type <= simplex_type) {
                    continue;
                }
                int8_t num_simplices = simplex_count(mesh_type, simplex_type);
                for (int8_t simplex_index = 0; simplex_index < num_simplices; ++simplex_index) {
                    // check basic functions are doing the right things


                    auto [act, size] = wmtk::autogen::utils::subdart_maximal_action_to_face(
                        mesh_type,
                        index,
                        simplex_type,
                        simplex_index);
                    auto a = wmtk::autogen::utils::subdart_maximal_action_to_face_action(
                        mesh_type,
                        index,
                        simplex_type,
                        simplex_index);
                    auto s = wmtk::autogen::utils::subdart_maximal_action_to_face_size(
                        mesh_type,
                        index,
                        simplex_type,
                        simplex_index);

                    REQUIRE(act == a);
                    REQUIRE(size == s);


                    int8_t mapped_index = sd.product(act, index);
                    CHECK(equal_subdart_dimension(mesh_type, index, mapped_index) == size);
                }
            }
        }
    }
}
