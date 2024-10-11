#include <array>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/internal/boundary_with_preserved_face.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"
#include "../tools/all_valid_local_tuples.hpp"

using namespace wmtk;
using namespace simplex;
using namespace tests;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("simplex_coface_preserving_boundary_tuples", "[simplex_collection]")
{
    // is a a face of b. includes if a == b (i.e implements <=)
    auto is_face = [&](const auto& m, const Simplex& a, const Simplex& b) -> bool {
        // compute b.faces() and then check if a is in it
        if (simplex::utils::SimplexComparisons::equal(m, a, b)) {
            return true;
        }
        const simplex::SimplexCollection faces = simplex::faces(m, b);
        return faces.contains(m.get_id_simplex(a));
    };

    auto run = [&](const auto& m,
                   const Simplex& base_s,
                   const Simplex& some_coface_s,
                   int64_t expected_size) {
        // amke sure that these simplices are actually the same thing
        REQUIRE(base_s.tuple() == some_coface_s.tuple());

        auto simplices = wmtk::simplex::internal::boundary_with_preserved_face_simplices(
            m,
            some_coface_s,
            base_s.primitive_type());

        REQUIRE(expected_size == simplices.size());
        for (const Simplex& ct : simplices) {
            // check that base_s <= ct <= some_coface_s
            CHECK(is_face(m, base_s, ct));
            CHECK(is_face(m, ct, some_coface_s));
        }
    };
    // TODOfix: commented out because of HalfeEdge removal
    //{
    //    tests::DEBUG_TriMesh m = tests::single_triangle();
    //    auto all_tuples = all_valid_local_tuples(PrimitiveType::Face);
    //
    //
    //    for (const Tuple& t : all_tuples) {
    //        // test for assert failure
    //        auto simplices = wmtk::simplex::internal::boundary_with_preserved_face_tuples(
    //            m,
    //            Simplex::face(t),
    //            PrimitiveType::HalfEdge);
    //
    //        run(m, Simplex(m,PV, t), Simplex(m,PV, t), 0);
    //        run(m, Simplex(m,PV, t), Simplex(m,PE, t), 1); // 1 vert
    //        run(m, Simplex(m,PV, t), Simplex(m,PF, t), 2); // 2 edges
    //
    //        run(m, Simplex(m,PE, t), Simplex(m,PV, t), 0);
    //        run(m, Simplex(m,PE, t), Simplex(m,PE, t), 0);
    //        run(m, Simplex(m,PE, t), Simplex(m,PF, t), 1); // 1 edge
    //
    //        run(m, Simplex(m,PF, t), Simplex(m,PV, t), 0);
    //        run(m, Simplex(m,PF, t), Simplex(m,PE, t), 0);
    //        run(m, Simplex(m,PF, t), Simplex(m,PF, t), 0);
    //    }
    //}
    {
        tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();
        auto all_tuples = all_valid_local_tuples(PrimitiveType::Tetrahedron);

        for (const Tuple& t : all_tuples) {
            run(m, Simplex(m, PV, t), Simplex(m, PV, t), 0);
            run(m, Simplex(m, PV, t), Simplex(m, PE, t), 1); // 1 vert
            run(m, Simplex(m, PV, t), Simplex(m, PF, t), 2); // 2 edges
            run(m, Simplex(m, PV, t), Simplex(m, PT, t), 3); // 3 faces

            run(m, Simplex(m, PE, t), Simplex(m, PV, t), 0);
            run(m, Simplex(m, PE, t), Simplex(m, PE, t), 0);
            run(m, Simplex(m, PE, t), Simplex(m, PF, t), 1); // 1 edge
            run(m, Simplex(m, PE, t), Simplex(m, PT, t), 2); // 2 faces

            run(m, Simplex(m, PF, t), Simplex(m, PV, t), 0);
            run(m, Simplex(m, PF, t), Simplex(m, PE, t), 0);
            run(m, Simplex(m, PF, t), Simplex(m, PF, t), 0);
            run(m, Simplex(m, PF, t), Simplex(m, PT, t), 1); // 1 face

            run(m, Simplex(m, PT, t), Simplex(m, PV, t), 0);
            run(m, Simplex(m, PT, t), Simplex(m, PE, t), 0);
            run(m, Simplex(m, PT, t), Simplex(m, PF, t), 0);
            run(m, Simplex(m, PT, t), Simplex(m, PT, t), 0);
        }
    }
}
