#include <wmtk/TriMesh.h>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/RawSimplexCollection.hpp>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace simplex;
using namespace wmtk::utils::examples;

TEST_CASE("raw_simplex_comparison", "[raw_simplex_collection]")
{
    // switching anything in a tuple besides the currently viewed simplex must not change the
    // simplex
    using Tuple = TriMesh::Tuple;

    tri::TriMeshVF VF = tri::two_triangles();
    TriMesh m;
    m.init(VF.F);

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_vertices();
        REQUIRE(vertices.size() == 4);
        for (const Tuple& t : vertices) {
            const RawSimplex<1> s0(t.vid(m));
            CHECK(s0.DIM == 0);
            const RawSimplex<1> s1(t.switch_edge(m).vid(m));
            CHECK(s1.DIM == 0);
            const RawSimplex<2> s_edge = m.simplex_from_edge(t);
            CHECK(s_edge.DIM == 1);

            CHECK(s0 == s1);
            CHECK_FALSE(s0 == s_edge);
            CHECK(s0 < s_edge);

            const std::optional<Tuple> t_opp = t.switch_face(m);
            if (!t_opp) {
                continue;
            }
            const RawSimplex<1> s2(t_opp.value().vid(m));
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_edges();
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const RawSimplex s0 = m.simplex_from_edge(t);
            const RawSimplex s1 = m.simplex_from_edge(t.switch_vertex(m));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);

            const std::optional<Tuple> t_opp = t.switch_face(m);
            if (!t_opp) {
                continue;
            }
            const RawSimplex s2 = m.simplex_from_edge(t_opp.value());
            CHECK_FALSE(s0 < s2);
            CHECK_FALSE(s2 < s0);
            CHECK_FALSE(s1 < s2);
            CHECK_FALSE(s2 < s1);
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_faces();
        REQUIRE(faces.size() == 2);
        for (const Tuple& t : faces) {
            const RawSimplex s0 = m.simplex_from_face(t);
            const RawSimplex s1 = m.simplex_from_face(t.switch_vertex(m));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            const RawSimplex s2 = m.simplex_from_face(t.switch_edge(m));
            CHECK_FALSE(s0 < s2);
            CHECK_FALSE(s2 < s0);
            CHECK_FALSE(s1 < s2);
            CHECK_FALSE(s2 < s1);
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
}

TEST_CASE("raw_simplex_collection_sorting", "[raw_simplex_collection]")
{
    using Tuple = TriMesh::Tuple;

    tri::TriMeshVF VF = tri::two_triangles();
    TriMesh m;
    m.init(VF.F);

    const std::vector<Tuple> vertices = m.get_vertices();
    CHECK(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_edges();
    CHECK(edges.size() == 5);
    const std::vector<Tuple> faces = m.get_faces();
    CHECK(faces.size() == 2);

    RawSimplexCollection raw_simplex_collection;
    for (const Tuple& t : vertices) {
        raw_simplex_collection.add(m.simplex_from_vertex(t));
    }
    for (const Tuple& t : edges) {
        raw_simplex_collection.add(m.simplex_from_edge(t));
    }
    for (const Tuple& t : faces) {
        raw_simplex_collection.add(m.simplex_from_face(t));
    }
    CHECK(raw_simplex_collection.vertices().size() == 4);
    CHECK(raw_simplex_collection.edges().size() == 5);
    CHECK(raw_simplex_collection.faces().size() == 2);
    CHECK(raw_simplex_collection.tets().empty());

    // test sorting and clean-up
    raw_simplex_collection.add(m.simplex_from_vertex(vertices[0]));
    raw_simplex_collection.add(m.simplex_from_edge(edges[0]));
    raw_simplex_collection.add(m.simplex_from_face(faces[0]));

    CHECK(raw_simplex_collection.vertices().size() == 5);
    CHECK(raw_simplex_collection.edges().size() == 6);
    CHECK(raw_simplex_collection.faces().size() == 3);
    CHECK(raw_simplex_collection.tets().empty());

    raw_simplex_collection.sort_and_clean();

    CHECK(raw_simplex_collection.vertices().size() == 4);
    CHECK(raw_simplex_collection.edges().size() == 5);
    CHECK(raw_simplex_collection.faces().size() == 2);
    CHECK(raw_simplex_collection.tets().empty());
}

TEST_CASE("raw_simplex_collection_set_operations", "[raw_simplex_collection]")
{
    RawSimplexCollection sc1, sc2;
    sc1.add(RawSimplex<1>(0));
    sc1.add(RawSimplex<2>(0, 1));
    sc1.add(RawSimplex<2>(2, 0));
    sc1.sort_and_clean();

    CHECK(sc1.contains(RawSimplex<1>(0)));
    CHECK(sc1.contains(RawSimplex<2>(0, 1)));
    CHECK(sc1.contains(RawSimplex<2>(0, 2)));
    CHECK_FALSE(sc1.contains(RawSimplex<1>(1)));

    sc2.add(RawSimplex<1>(1));
    {
        auto inters = RawSimplexCollection::get_intersection(sc1, sc2);
        CHECK(inters.vertices().empty());
        CHECK(inters.edges().empty());
        CHECK(inters.faces().empty());
        CHECK(inters.tets().empty());
    }

    RawSimplexCollection sc_union1 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(sc_union1.size() == sc1.size() + sc2.size());
    CHECK(sc_union1.contains(RawSimplex<1>(0)));
    CHECK(sc_union1.contains(RawSimplex<1>(1)));

    sc2.add(RawSimplex<1>(0));
    sc2.sort_and_clean();

    RawSimplexCollection sc_union2 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(RawSimplexCollection::are_simplex_collections_equal(sc_union1, sc_union2));

    RawSimplexCollection sc_inter = RawSimplexCollection::get_intersection(sc1, sc2);
    CHECK(sc_inter.size() == 1);
    CHECK(sc_inter.contains(RawSimplex<1>(0)));
}

TEST_CASE("raw_simplex_faces", "[raw_simplex_collection]")
{
    SECTION("without_mesh")
    {
        RawSimplex<4> tet(0, 1, 2, 3);
        CHECK(tet.DIM == 3);
        RawSimplexCollection tet_faces = RawSimplexCollection::faces_from_simplex(tet);
        CHECK(tet_faces.size() == 14);
        CHECK(tet_faces.contains(RawSimplex<1>(0)));
        CHECK(tet_faces.contains(RawSimplex<1>(1)));
        CHECK(tet_faces.contains(RawSimplex<1>(2)));
        CHECK(tet_faces.contains(RawSimplex<1>(3)));
        CHECK(tet_faces.contains(RawSimplex<2>(0, 1)));
        CHECK(tet_faces.contains(RawSimplex<2>(0, 2)));
        CHECK(tet_faces.contains(RawSimplex<2>(0, 3)));
        CHECK(tet_faces.contains(RawSimplex<2>(1, 2)));
        CHECK(tet_faces.contains(RawSimplex<2>(1, 3)));
        CHECK(tet_faces.contains(RawSimplex<2>(2, 3)));
        CHECK(tet_faces.contains(RawSimplex<3>(0, 1, 2)));
        CHECK(tet_faces.contains(RawSimplex<3>(0, 1, 3)));
        CHECK(tet_faces.contains(RawSimplex<3>(0, 2, 3)));
        CHECK(tet_faces.contains(RawSimplex<3>(1, 2, 3)));

        auto tri = tet.opposite_face(0);
        CHECK(tri.DIM == 2);
        CHECK(tri < tet);

        RawSimplexCollection tri_faces = RawSimplexCollection::faces_from_simplex(tri);
        CHECK(tri_faces.size() == 6);
        CHECK(tri_faces.contains(RawSimplex<1>(1)));
        CHECK(tri_faces.contains(RawSimplex<1>(2)));
        CHECK(tri_faces.contains(RawSimplex<1>(3)));
        CHECK(tri_faces.contains(RawSimplex<2>(1, 2)));
        CHECK(tri_faces.contains(RawSimplex<2>(1, 3)));
        CHECK(tri_faces.contains(RawSimplex<2>(2, 3)));

        auto edge = tri.opposite_face(2);
        CHECK(edge.DIM == 1);
        RawSimplexCollection edge_faces = RawSimplexCollection::faces_from_simplex(edge);
        CHECK(edge_faces.size() == 2);
        CHECK(edge_faces.contains(RawSimplex<1>(1)));
        CHECK(edge_faces.contains(RawSimplex<1>(3)));

        auto vertex = edge.opposite_face(3);
        CHECK(vertex.DIM == 0);
        CHECK((RawSimplexCollection::faces_from_simplex(vertex)).size() == 0);

        auto opp_edge = tet.opposite_face(edge);
        CHECK(opp_edge.DIM == 1);
        RawSimplexCollection opp_edge_faces = RawSimplexCollection::faces_from_simplex(opp_edge);
        CHECK(opp_edge_faces.size() == 2);
        CHECK(opp_edge_faces.contains(RawSimplex<1>(0)));
        CHECK(opp_edge_faces.contains(RawSimplex<1>(2)));

        auto opp_vertex = tet.opposite_face(tri);
        CHECK(opp_vertex.DIM == 0);
        CHECK((RawSimplexCollection::faces_from_simplex(opp_vertex)).size() == 0);
    }
    SECTION("with_mesh")
    {
        using Tuple = TriMesh::Tuple;

        tri::TriMeshVF VF = tri::single_triangle();
        TriMesh m;
        m.init(VF.F);

        RawSimplexCollection sc;
        for (const Tuple& t : m.get_edges()) {
            RawSimplex<3> s(t.vid(m), t.switch_vertex(m).vid(m), -1);
            CHECK(s.DIM == 2);
            RawSimplexCollection s_faces = RawSimplexCollection::faces_from_simplex(s);
            CHECK(s_faces.size() == 6);
            CHECK(s_faces.contains(RawSimplex<1>(-1)));

            auto opposite_edge = s.opposite_face(t.vid(m));
            CHECK(opposite_edge.DIM == 1);
            auto oef = RawSimplexCollection::faces_from_simplex(opposite_edge);
            CHECK(oef.contains(RawSimplex<1>(-1)));

            sc.add(s);
        }

        CHECK(sc.size() == 3);
        sc.sort_and_clean();
        CHECK(sc.size() == 3);
    }
}
