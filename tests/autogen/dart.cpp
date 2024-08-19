#include <catch2/catch_test_macros.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/is_ccw.hpp>
#include <wmtk/autogen/local_dart_action.hpp>
#include <wmtk/autogen/local_switch_tuple.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "tools/all_valid_local_tuples.hpp"
using namespace wmtk;
using namespace wmtk::autogen;
using namespace wmtk::tests;

TEST_CASE("tuple_autogen_index_dart_tuple_conversion", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        auto tuples = wmtk::tests::all_valid_local_tuples(mesh_type);
        autogen::SimplexDart sd(mesh_type);

        for (const auto& t : tuples) {
            int8_t i = sd.valid_index_from_tuple(t);
            Tuple nt = sd.tuple_from_valid_index(0, i);

            CHECK(t == nt);
        }
    }
}

TEST_CASE("tuple_autogen_index_dart_group_structure", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        autogen::SimplexDart sd(mesh_type);
        assert(size_t(sd.valid_indices().size()) == sd.size());

        for (PrimitiveType pt : wmtk::utils::primitive_below(mesh_type)) {
            const int8_t index_switch = sd.primitive_as_index(pt);
            CHECK(sd.identity() == sd.product(index_switch, index_switch));
            CHECK(index_switch == sd.inverse(index_switch));
        }
        for (int8_t index = 0; index < sd.size(); ++index) {
            const int8_t inv = sd.inverse(index);
            CHECK(sd.product(index, inv) == sd.identity());
            CHECK(sd.product(inv, index) == sd.identity());
            // for (int8_t index2 = 0; index2 < sd.size(); ++index2) {
            //     const int8_t inv2 = sd.inverse(index2);
            //     const int8_t p = sd.product(index, index2);
            //     const int8_t pi = sd.product(inv2, inv);
            //     CHECK(pi == sd.inverse(p));
            //     CHECK(sd.product(p, pi) == sd.identity());
            //     CHECK(sd.product(pi, p) == sd.identity());
            // }
        }
    }
}


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
    // when other meshes are available add them here
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
                REQUIRE(sd.convert(index, sd2) != -1);
                REQUIRE(sd2.convert(sd.convert(index, sd2), sd) == index);
            }

            for (int8_t index = 0; index < sd.size(); ++index) {
                const int8_t inv = sd.inverse(index);

                int8_t index2 = sd.convert(index, sd2);
                int8_t inv2 = sd.convert(inv, sd2);
                CHECK(sd2.product(index2, inv2) == sd2.identity());
            }
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
