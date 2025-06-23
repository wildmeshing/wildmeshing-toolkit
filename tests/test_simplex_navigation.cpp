#include <wmtk/TriMesh.h>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace simplex;
using namespace wmtk::utils::examples;

TEST_CASE("tri_simplex_incident_triangles", "[raw_simplex_collection]")
{
    using Tuple = TriMesh::Tuple;

    tri::TriMeshVF VF = tri::edge_region();
    TriMesh m;
    m.init(VF.F);
    // boundary vertex
    {
        const auto sc = m.simplex_incident_triangles(Vertex(0));
        CHECK(sc.size() == 2);
        REQUIRE(sc.faces().size() == 2);
        const Tuple f0 = m.tuple_from_simplex(sc.faces()[0]);
        CHECK(f0.fid(m) == 1);
        const Tuple f1 = m.tuple_from_simplex(sc.faces()[1]);
        CHECK(f1.fid(m) == 0);
    }
    // interior vertex
    {
        const auto sc = m.simplex_incident_triangles(Vertex(4));
        CHECK(sc.size() == 6);
        REQUIRE(sc.faces().size() == 6);
        std::set<size_t> fids;
        for (const Face& f : sc.faces()) {
            fids.insert(m.tuple_from_simplex(f).fid(m));
        }
        std::set<size_t> comp{0, 1, 2, 5, 6, 7};
        CHECK(fids == comp);
    }
    // boundary edge
    {
        const Tuple t = m.tuple_from_vids(0, 1, 4);
        const Edge e = m.simplex_from_edge(t);
        const auto sc = m.simplex_incident_triangles(e);
        CHECK(sc.size() == 1);
        REQUIRE(sc.faces().size() == 1);
        CHECK(m.tuple_from_simplex(sc.faces()[0]).fid(m) == 1);
    }
    // interior edge
    {
        const Tuple t = m.tuple_from_vids(4, 5, 1);
        const Edge e = m.simplex_from_edge(t);
        const auto sc = m.simplex_incident_triangles(e);
        CHECK(sc.size() == 2);
        REQUIRE(sc.faces().size() == 2);
        CHECK(m.tuple_from_simplex(sc.faces()[0]).fid(m) == 2);
        CHECK(m.tuple_from_simplex(sc.faces()[1]).fid(m) == 7);
    }
}

TEST_CASE("tri_simplex_link", "[raw_simplex_collection]")
{
    using Tuple = TriMesh::Tuple;

    tri::TriMeshVF VF = tri::edge_region();
    TriMesh m;
    m.init(VF.F);
    // boundary vertex - vertices
    {
        const auto sc = m.simplex_link_vertices(Vertex(0));
        CHECK(sc.size() == 3);
        REQUIRE(sc.vertices().size() == 3);
        CHECK(sc.vertices()[0].vertices()[0] == 1);
        CHECK(sc.vertices()[1].vertices()[0] == 3);
        CHECK(sc.vertices()[2].vertices()[0] == 4);
    }
    // interior vertex - vertices
    {
        const auto sc = m.simplex_link_vertices(Vertex(4));
        CHECK(sc.size() == 6);
        REQUIRE(sc.vertices().size() == 6);
        CHECK(sc.vertices()[0].vertices()[0] == 0);
        CHECK(sc.vertices()[1].vertices()[0] == 1);
        CHECK(sc.vertices()[2].vertices()[0] == 3);
        CHECK(sc.vertices()[3].vertices()[0] == 5);
        CHECK(sc.vertices()[4].vertices()[0] == 7);
        CHECK(sc.vertices()[5].vertices()[0] == 8);
    }
    // boundary edge - vertices
    {
        const Tuple t = m.tuple_from_vids(0, 1, 4);
        const Edge e = m.simplex_from_edge(t);
        const auto sc = m.simplex_link_vertices(e);
        CHECK(sc.size() == 1);
        REQUIRE(sc.vertices().size() == 1);
        CHECK(sc.vertices()[0].vertices()[0] == 4);
    }
    // interior edge - vertices
    {
        const Tuple t = m.tuple_from_vids(4, 5, 1);
        const Edge e = m.simplex_from_edge(t);
        const auto sc = m.simplex_link_vertices(e);
        CHECK(sc.size() == 2);
        REQUIRE(sc.vertices().size() == 2);
        CHECK(sc.vertices()[0].vertices()[0] == 1);
        CHECK(sc.vertices()[1].vertices()[0] == 8);
    }
    // boundary vertex - edges
    {
        const auto sc = m.simplex_link_edges(Vertex(0));
        CHECK(sc.size() == 2);
        REQUIRE(sc.edges().size() == 2);
        RawSimplexCollection comp;
        comp.add(Edge(1, 4));
        comp.add(Edge(3, 4));
        CHECK(RawSimplexCollection::are_simplex_collections_equal(sc, comp));
    }
    // interior vertex - edges
    {
        const auto sc = m.simplex_link_edges(Vertex(4));
        CHECK(sc.size() == 6);
        REQUIRE(sc.edges().size() == 6);
        RawSimplexCollection comp;
        comp.add(Edge(0, 1));
        comp.add(Edge(1, 5));
        comp.add(Edge(5, 8));
        comp.add(Edge(8, 7));
        comp.add(Edge(7, 3));
        comp.add(Edge(3, 0));
        comp.sort_and_clean();
        CHECK(RawSimplexCollection::are_simplex_collections_equal(sc, comp));
    }
}