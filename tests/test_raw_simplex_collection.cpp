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
            const Vertex s0(t.vid(m));
            const Vertex s1(t.switch_edge(m).vid(m));
            const Edge s_edge = m.simplex_from_edge(t);

            CHECK(s0 == s1);

            const std::optional<Tuple> t_opp = t.switch_face(m);
            if (!t_opp) {
                continue;
            }
            const Vertex s2(t_opp.value().vid(m));
            CHECK(s0 == s2);
            CHECK(s1 == s2);
        }
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_edges();
        REQUIRE(edges.size() == 5);
        for (const Tuple& t : edges) {
            const Edge s0 = m.simplex_from_edge(t);
            const Edge s1 = m.simplex_from_edge(t.switch_vertex(m));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);

            const std::optional<Tuple> t_opp = t.switch_face(m);
            if (!t_opp) {
                continue;
            }
            const Edge s2 = m.simplex_from_edge(t_opp.value());
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
            const Face s0 = m.simplex_from_face(t);
            const Face s1 = m.simplex_from_face(t.switch_vertex(m));
            CHECK_FALSE(s0 < s1);
            CHECK_FALSE(s1 < s0);
            CHECK(s0 == s1);
            const Face s2 = m.simplex_from_face(t.switch_edge(m));
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
    sc1.add(Vertex(0));
    sc1.add(Edge(0, 1));
    sc1.add(Edge(2, 0));
    sc1.sort_and_clean();

    CHECK(sc1.contains(Vertex(0)));
    CHECK(sc1.contains(Edge(0, 1)));
    CHECK(sc1.contains(Edge(0, 2)));
    CHECK_FALSE(sc1.contains(Vertex(1)));

    sc2.add(Vertex(1));
    {
        auto inters = RawSimplexCollection::get_intersection(sc1, sc2);
        CHECK(inters.vertices().empty());
        CHECK(inters.edges().empty());
        CHECK(inters.faces().empty());
        CHECK(inters.tets().empty());
    }

    RawSimplexCollection sc_union1 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(sc_union1.size() == sc1.size() + sc2.size());
    CHECK(sc_union1.contains(Vertex(0)));
    CHECK(sc_union1.contains(Vertex(1)));

    sc2.add(Vertex(0));
    sc2.sort_and_clean();

    RawSimplexCollection sc_union2 = RawSimplexCollection::get_union(sc1, sc2);
    CHECK(RawSimplexCollection::are_simplex_collections_equal(sc_union1, sc_union2));

    RawSimplexCollection sc_inter = RawSimplexCollection::get_intersection(sc1, sc2);
    CHECK(sc_inter.size() == 1);
    CHECK(sc_inter.contains(Vertex(0)));
}

TEST_CASE("raw_simplex_faces", "[raw_simplex_collection]")
{
    SECTION("without_mesh")
    {
        Tet tet(0, 1, 2, 3);
        RawSimplexCollection tet_faces = RawSimplexCollection::faces_from_simplex(tet);
        CHECK(tet_faces.size() == 14);
        CHECK(tet_faces.contains(Vertex(0)));
        CHECK(tet_faces.contains(Vertex(1)));
        CHECK(tet_faces.contains(Vertex(2)));
        CHECK(tet_faces.contains(Vertex(3)));
        CHECK(tet_faces.contains(Edge(0, 1)));
        CHECK(tet_faces.contains(Edge(0, 2)));
        CHECK(tet_faces.contains(Edge(0, 3)));
        CHECK(tet_faces.contains(Edge(1, 2)));
        CHECK(tet_faces.contains(Edge(1, 3)));
        CHECK(tet_faces.contains(Edge(2, 3)));
        CHECK(tet_faces.contains(Face(0, 1, 2)));
        CHECK(tet_faces.contains(Face(0, 1, 3)));
        CHECK(tet_faces.contains(Face(0, 2, 3)));
        CHECK(tet_faces.contains(Face(1, 2, 3)));

        auto tri = tet.opposite_face(0);

        RawSimplexCollection tri_faces = RawSimplexCollection::faces_from_simplex(tri);
        CHECK(tri_faces.size() == 6);
        CHECK(tri_faces.contains(Vertex(1)));
        CHECK(tri_faces.contains(Vertex(2)));
        CHECK(tri_faces.contains(Vertex(3)));
        CHECK(tri_faces.contains(Edge(1, 2)));
        CHECK(tri_faces.contains(Edge(1, 3)));
        CHECK(tri_faces.contains(Edge(2, 3)));

        auto edge = tri.opposite_edge(2);
        RawSimplexCollection edge_faces = RawSimplexCollection::faces_from_simplex(edge);
        CHECK(edge_faces.size() == 2);
        CHECK(edge_faces.contains(Vertex(1)));
        CHECK(edge_faces.contains(Vertex(3)));

        auto vertex = edge.opposite_vertex(3);
        CHECK((RawSimplexCollection::faces_from_simplex(vertex)).size() == 0);

        auto opp_edge = tet.opposite_edge(edge);
        RawSimplexCollection opp_edge_faces = RawSimplexCollection::faces_from_simplex(opp_edge);
        CHECK(opp_edge_faces.size() == 2);
        CHECK(opp_edge_faces.contains(Vertex(0)));
        CHECK(opp_edge_faces.contains(Vertex(2)));

        auto opp_vertex = tet.opposite_vertex(tri);
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
            Face s(t.vid(m), t.switch_vertex(m).vid(m), -1);
            RawSimplexCollection s_faces = RawSimplexCollection::faces_from_simplex(s);
            CHECK(s_faces.size() == 6);
            CHECK(s_faces.contains(Vertex(-1)));

            auto opposite_edge = s.opposite_edge(t.vid(m));
            auto oef = RawSimplexCollection::faces_from_simplex(opposite_edge);
            CHECK(oef.contains(Vertex(-1)));

            sc.add(s);
        }

        CHECK(sc.size() == 3);
        sc.sort_and_clean();
        CHECK(sc.size() == 3);
    }
}
