#include <catch2/catch_test_macros.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "tools/EdgeMesh_examples.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"


TEST_CASE("test_mesh_boundary", "[mesh]")
{
    wmtk::PointMesh pm(10);
    wmtk::EdgeMesh em = wmtk::tests::single_line();
    wmtk::TriMesh fm = wmtk::tests::single_triangle();
    wmtk::TetMesh tm = wmtk::tests_3d::single_tet();

    CHECK_THROWS(pm.is_boundary(wmtk::PrimitiveType::Edge, {}));
    CHECK_THROWS(pm.is_boundary(wmtk::PrimitiveType::Triangle, {}));
    CHECK_THROWS(pm.is_boundary(wmtk::PrimitiveType::Tetrahedron, {}));
    // CHECK_THROWS(pm.is_boundary(wmtk::PrimitiveType::HalfEdge, {}));

    CHECK_THROWS(em.is_boundary(wmtk::PrimitiveType::Triangle, {}));
    CHECK_THROWS(em.is_boundary(wmtk::PrimitiveType::Tetrahedron, {}));
    // CHECK_THROWS(em.is_boundary(wmtk::PrimitiveType::HalfEdge, {}));

    CHECK_THROWS(fm.is_boundary(wmtk::PrimitiveType::Tetrahedron, {}));
    // CHECK_THROWS(fm.is_boundary(wmtk::PrimitiveType::HalfEdge, {}));

    // CHECK_THROWS(tm.is_boundary(wmtk::PrimitiveType::HalfEdge, {}));
}
