#include <wmtk/TriMesh.h>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/utils/examples/TriMesh_examples.hpp>

using namespace wmtk;
using namespace simplex;
using namespace wmtk::utils::examples;

TEST_CASE("simplex_incident_triangles", "[raw_simplex_collection]")
{
    using Tuple = TriMesh::Tuple;

    tri::TriMeshVF VF = tri::edge_region();
    TriMesh m;
    m.init(VF.F);
    // boundary vertex
    {
        const auto sc = m.simplex_incident_triangles(Vertex(0));
        REQUIRE(sc.size() == 2);
        const Tuple f0 = m.tuple_from_simplex(sc.faces()[0]);
        CHECK(f0.fid(m) == 1);
        const Tuple f1 = m.tuple_from_simplex(sc.faces()[1]);
        CHECK(f1.fid(m) == 0);
    }
    // interior vertex
    {
        const auto sc = m.simplex_incident_triangles(Vertex(4));
        REQUIRE(sc.size() == 6);
        std::set<size_t> fids;
        for (const Face& f : sc.faces()) {
            fids.insert(m.tuple_from_simplex(f).fid(m));
        }
        std::set<size_t> comp{0, 1, 2, 5, 6, 7};
        CHECK(fids == comp);
    }
}
