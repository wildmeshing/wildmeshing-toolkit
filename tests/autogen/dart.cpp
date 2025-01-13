#include <catch2/catch_test_macros.hpp>
#include <numeric>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/SimplexAdjacency.hpp>
#include <wmtk/autogen/utils/local_id_table_offset.hpp>
#include <wmtk/autogen/utils/simplex_index_from_valid_index.hpp>

#include <wmtk/utils/TupleInspector.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "tools/all_valid_local_tuples.hpp"
#include "tools/darts_using_faces.hpp"
using namespace wmtk;
using namespace wmtk::autogen;
using namespace wmtk::tests;

namespace {

    template <int D>
    void check_sa_dim() {
    static_assert(sizeof(SimplexAdjacency<D>) <= sizeof(int64_t) * (D+1));
    }
}
TEST_CASE("simplex_adjacency_io", "[dart]") {
    SimplexAdjacency<3> sa;
    check_sa_dim<1>();
    check_sa_dim<2>();
    check_sa_dim<3>();

    sa[0] = Dart(-1,1);
    sa[1] = Dart(0,2);
    sa[2] = Dart(5,3);


    CHECK(sa[0].global_id() == -1);
    CHECK(sa[1].global_id() == 0);
    CHECK(sa[2].global_id() == 5);
    CHECK(sa[0].local_orientation() == 1);
    CHECK(sa[1].local_orientation() == 2);
    CHECK(sa[2].local_orientation() == 3);

    auto saf = sa[0];
    CHECK(saf.global_id() == -1);
    CHECK(saf.local_orientation() == 5);

    saf = Dart(20,30);
    CHECK(sa[0].global_id() == 20);
    CHECK(sa[0].local_orientation() == 30);
}



TEST_CASE("tuple_autogen_valid_indices_equal", "[tuple]")
{
    // when other meshes are available add them here
    for (PrimitiveType mesh_type :
         {PrimitiveType::Edge, PrimitiveType::Triangle, PrimitiveType::Tetrahedron}) {
        auto tuples = wmtk::tests::all_valid_local_tuples(mesh_type);
        autogen::SimplexDart sd(mesh_type);


        std::vector<int8_t> indices_from_tuples, valid_indices_from_tuples;
        for (const auto& t : tuples) {
            indices_from_tuples.emplace_back(sd.valid_index_from_tuple(t));
            valid_indices_from_tuples.emplace_back(
                wmtk::autogen::utils::local_id_table_offset(mesh_type, t));
        }
        VectorX<int8_t> valid_indices = sd.valid_indices();
        std::vector<int8_t> range(valid_indices.size());
        std::iota(range.begin(), range.end(), 0);
        CHECK(range == indices_from_tuples);

        std::vector<int8_t> valid_indices_vec(valid_indices.begin(), valid_indices.end());

        CHECK(valid_indices_vec == valid_indices_from_tuples);
    }
}

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

            for (PrimitiveType pt = PrimitiveType::Vertex; pt < mesh_type; pt = pt + 1) {
                CHECK(
                    sd.simplex_index(i, pt) ==
                    wmtk::autogen::utils::simplex_index_from_valid_index(mesh_type, i, pt));
                CHECK(sd.simplex_index(i, pt) == wmtk::utils::TupleInspector::local_id(nt, pt));
            }


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

