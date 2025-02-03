#include <catch2/catch_test_macros.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

#include "tools/DEBUG_EdgeMesh.hpp"

using namespace wmtk;
using namespace submesh;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("submesh_init", "[mesh][submesh]")
{
    // logger().set_level(spdlog::level::off);

    // basic test for implementing
    std::shared_ptr<tests::DEBUG_TriMesh> mesh_in =
        std::make_shared<tests::DEBUG_TriMesh>(tests::edge_region_with_position());

    tests::DEBUG_TriMesh& m = *mesh_in;
    const Tuple edge45 = m.edge_tuple_from_vids(4, 5);

    Embedding emb(mesh_in);
    std::shared_ptr<SubMesh> sub_ptr = emb.add_submesh();
    SubMesh& sub = *sub_ptr;

    CHECK_THROWS(sub.top_simplex_type());

    sub.add_simplex(edge45, PE);

    CHECK(sub.contains(edge45, PrimitiveType::Edge));
    CHECK(sub.contains(edge45, PrimitiveType::Vertex));
    CHECK(sub.contains(sub.switch_tuple(edge45, PV), PrimitiveType::Vertex));
    CHECK(sub.top_simplex_type(edge45) == PrimitiveType::Edge);

    {
        ParaviewWriter writer("submesh_init", "vertices", m, false, true, true, false);
        CHECK_NOTHROW(m.serialize(writer));
    }
}
