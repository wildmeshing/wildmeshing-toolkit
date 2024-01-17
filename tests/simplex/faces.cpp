
#include "tools/TetMesh_examples.hpp"
#include <wmtk/simplex/SimplexCollection.hpp>
#include <catch2/catch_test_macros.hpp>
#include "tools/TriMesh_examples.hpp"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_iterable.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
using namespace wmtk;
using namespace simplex;
TEST_CASE("simplex_faces", "[simplex_collection][2D]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));

    SECTION("vertex")
    {
        SimplexCollection bd = faces(m, Simplex::vertex(t));
        REQUIRE(bd.simplex_vector().size() == 0);
    }
    SECTION("edge")
    {
        SimplexCollection bd = faces(m, Simplex::edge(t));
        REQUIRE(bd.simplex_vector().size() == 2);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        REQUIRE(v.size() == 2);
        CHECK(m.id(v[0]) == 0);
        CHECK(m.id(v[1]) == 1);

        CHECK(bd.simplex_vector(PrimitiveType::Edge).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("face")
    {
        SimplexCollection bd = faces(m, Simplex::face(t));
        REQUIRE(bd.simplex_vector().size() == 6);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 3);
        for (size_t i = 0; i < v.size(); ++i) {
            CHECK(m.id(v[i]) == i);
        }

        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 3);
        SimplexCollection expected_edges(m);
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(0, 1, 0)));
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(1, 2, 0)));
        expected_edges.add(Simplex::edge(m.edge_tuple_between_v1_v2(2, 0, 0)));
        expected_edges.sort_and_clean();
        const std::vector<Simplex> expected_edge_simplices =
            expected_edges.simplex_vector(PrimitiveType::Edge);
        REQUIRE(e.size() <= 3);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(simplex::utils::SimplexComparisons::equal(m, e[i], expected_edge_simplices[i]));
        }

        CHECK(bd.simplex_vector(PrimitiveType::Face).size() == 0);
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
    SECTION("tetrahedron")
    {
        SimplexCollection bd = faces(m, Simplex::tetrahedron(t));
        REQUIRE(bd.simplex_vector().size() == 14);
        const std::vector<Simplex> v = bd.simplex_vector(PrimitiveType::Vertex);
        CHECK(v.size() == 4);
        for (size_t i = 0; i < v.size(); ++i) {
            CHECK(m.id(v[i]) == i);
        }

        const std::vector<Simplex> e = bd.simplex_vector(PrimitiveType::Edge);
        CHECK(e.size() == 6);
        for (size_t i = 0; i < e.size(); ++i) {
            CHECK(m.id(e[i]) == i);
        }


        const std::vector<Simplex> f = bd.simplex_vector(PrimitiveType::Face);
        CHECK(f.size() == 4);
        for (size_t i = 0; i < f.size(); ++i) {
            CHECK(m.id(f[i]) == i);
        }
        CHECK(bd.simplex_vector(PrimitiveType::Tetrahedron).size() == 0);
    }
}
TEST_CASE("simplex_faces_iterable", "[simplex_collection][2D]")
{
    tests::DEBUG_TriMesh m = tests::single_triangle();

    Simplex simplex = Simplex::vertex({});

    const Tuple t = m.edge_tuple_between_v1_v2(0, 1, 0);

    SECTION("vertex")
    {
        simplex = Simplex::vertex(t);
    }
    SECTION("edge")
    {
        simplex = Simplex::edge(t);
    }
    SECTION("face")
    {
        simplex = Simplex::face(t);
    }

    FacesIterable itrb = faces_iterable(m, simplex);
    SimplexCollection coll = faces(m, simplex);

    SimplexCollection itrb_collection(m);
    for (const Simplex& s : itrb) {
        itrb_collection.add(s);
    }
    itrb_collection.sort_and_clean();

    REQUIRE(itrb_collection.simplex_vector().size() == coll.simplex_vector().size());

    for (size_t i = 0; i < coll.simplex_vector().size(); ++i) {
        CHECK(simplex::utils::SimplexComparisons::equal(
            m,
            itrb_collection.simplex_vector()[i],
            coll.simplex_vector()[i]));
    }
}
TEST_CASE("simplex_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    SECTION("vertices_0123")
    {
        const std::vector<int64_t> expected_vids{0, 1, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("vertices_1023")
    {
        const std::vector<int64_t> expected_vids{1, 0, 2, 3};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(1, 0, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Vertex);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 2);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                CHECK(m.id(Simplex::vertex(vs[i])) == expected_vids[i]);
            }
        }
    }
    SECTION("edges_0123")
    {
        const std::vector<std::array<int64_t, 2>>
            expected_vids{{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Edge);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 3);
        CHECK(single_dim_faces[3].size() == 6);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices =
                    faces_single_dimension_tuples(m, Simplex::edge(vs[i]), PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 2);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const int64_t ev = m.id(Simplex::vertex(edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
    SECTION("faces_0123")
    {
        const std::vector<std::array<int64_t, 3>> expected_vids{
            {0, 1, 2},
            {0, 3, 1},
            {1, 3, 2},
            {2, 3, 0}};
        std::array<std::vector<Tuple>, 4> single_dim_faces;

        const Tuple t = m.switch_face(m.edge_tuple_between_v1_v2(0, 1, 0));
        for (int64_t dim = 0; dim < single_dim_faces.size(); ++dim) {
            single_dim_faces[dim] = faces_single_dimension_tuples(
                m,
                Simplex(get_primitive_type_from_id(dim), t),
                PrimitiveType::Face);
        }
        CHECK(single_dim_faces[0].size() == 0);
        CHECK(single_dim_faces[1].size() == 0);
        CHECK(single_dim_faces[2].size() == 0);
        CHECK(single_dim_faces[3].size() == 4);

        for (const std::vector<Tuple>& vs : single_dim_faces) {
            for (size_t i = 0; i < vs.size(); ++i) {
                const std::vector<Tuple> edge_vertices =
                    faces_single_dimension_tuples(m, Simplex::face(vs[i]), PrimitiveType::Vertex);
                CHECK(edge_vertices.size() == 3);
                for (size_t j = 0; j < edge_vertices.size(); ++j) {
                    const int64_t ev = m.id(Simplex::vertex(edge_vertices[j]));
                    CHECK(ev == expected_vids[i][j]);
                }
            }
        }
    }
}

TEST_CASE("simplex_compare_faces_with_faces_single_dimension", "[simplex_collection]")
{
    tests_3d::DEBUG_TetMesh m = tests_3d::single_tet();

    for (int64_t cell_dim = 0; cell_dim < 4; ++cell_dim) {
        const PrimitiveType cell_type = get_primitive_type_from_id(cell_dim);

        const std::vector<Tuple> cells = m.get_all(cell_type);

        for (int64_t face_dim = 0; face_dim < 4; ++face_dim) {
            const PrimitiveType face_type = get_primitive_type_from_id(face_dim);

            for (const Tuple& cell : cells) {
                const Simplex cell_simplex = Simplex(cell_type, cell);
                const std::vector<Tuple> fsd_vec =
                    faces_single_dimension_tuples(m, cell_simplex, face_type);

                SimplexCollection face_collection(m);
                for (const Tuple& f : fsd_vec) {
                    face_collection.add(Simplex(face_type, f));
                }
                face_collection.sort_and_clean();
                const std::vector<Simplex> faces_single_dim =
                    face_collection.simplex_vector(face_type);

                SimplexCollection bndry = faces(m, cell_simplex);
                const std::vector<Simplex> bndry_single_dim = bndry.simplex_vector(face_type);

                REQUIRE(faces_single_dim.size() == bndry_single_dim.size());
                for (size_t i = 0; i < faces_single_dim.size(); ++i) {
                    CHECK(simplex::utils::SimplexComparisons::equal(
                        m,
                        bndry_single_dim[i],
                        faces_single_dim[i]));
                }
            }
        }
    }
}
