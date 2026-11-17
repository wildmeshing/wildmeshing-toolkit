// SizingMesh: the target edge length of a sizing_mesh option, interpolated in its cells.
#include <wmtk/utils/SizingMesh.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace wmtk;

namespace {

MeshData with_field(MeshData m, const std::vector<double>& length)
{
    DataMatrix<double> l(length.size(), 1);
    for (size_t i = 0; i < length.size(); ++i) l(i, 0) = length[i];
    m.vertex_attributes[SizingMesh::field_name].values = l;
    return m;
}

/// The unit square as two triangles, at z = `z`.
MeshData square(double z = 0)
{
    MeshData m;
    m.V.resize(4, 3);
    m.V << 0, 0, z, 1, 0, z, 1, 1, z, 0, 1, z;
    m.cells[CellType::Triangle].connectivity.resize(2, 3);
    m.cells[CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    return m;
}

} // namespace

TEST_CASE("sizing-mesh-planar", "[sizing_mesh]")
{
    // Linear in x: 1 on the left, 3 on the right, exact inside both triangles.
    const SizingMesh s(with_field(square(), {1, 3, 3, 1}), SizingMesh::Kind::Planar);
    CHECK(s(Eigen::Vector2d(0, 0)) == Catch::Approx(1));
    CHECK(s(Eigen::Vector2d(0.25, 0.75)) == Catch::Approx(1.5));
    CHECK(s(Eigen::Vector2d(0.5, 0.1)) == Catch::Approx(2));
    CHECK(s(Eigen::Vector2d(1, 1)) == Catch::Approx(3));
    CHECK(s(Eigen::Vector2d(1.5, 0.5)) == -1); // not covered
    CHECK(s.min_length() == 1);
    CHECK(s.max_length() == 3);
}

TEST_CASE("sizing-mesh-volume", "[sizing_mesh]")
{
    MeshData m;
    m.V.resize(4, 3);
    m.V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    m.cells[CellType::Tetrahedron].connectivity.resize(1, 4);
    m.cells[CellType::Tetrahedron].connectivity << 0, 1, 2, 3;
    // 1 + x + 2y + 3z.
    const SizingMesh s(with_field(m, {1, 2, 3, 4}), SizingMesh::Kind::Volume);
    CHECK(s(Eigen::Vector3d(0.1, 0.2, 0.3)) == Catch::Approx(1 + 0.1 + 0.4 + 0.9));
    CHECK(s(Eigen::Vector3d(0.25, 0.25, 0.25)) == Catch::Approx(2.5));
    CHECK(s(Eigen::Vector3d(1, 1, 1)) == -1); // outside the tet
}

TEST_CASE("sizing-mesh-surface", "[sizing_mesh]")
{
    // At the closest point of the square at z = 0: off the surface, as on it.
    const SizingMesh s(with_field(square(), {1, 3, 3, 1}), SizingMesh::Kind::Surface);
    CHECK(s(Eigen::Vector3d(0.5, 0.1, 0)) == Catch::Approx(2));
    CHECK(s(Eigen::Vector3d(0.5, 0.1, 0.7)) == Catch::Approx(2));
    CHECK(s(Eigen::Vector3d(2, 0.5, -1)) == Catch::Approx(3)); // closest: the right side
}

TEST_CASE("sizing-mesh-errors", "[sizing_mesh]")
{
    using K = SizingMesh::Kind;
    CHECK_THROWS(SizingMesh(square(), K::Planar)); // no field
    CHECK_THROWS(SizingMesh(with_field(square(), {1, 1, 1}), K::Planar)); // not one per vertex
    CHECK_THROWS(SizingMesh(with_field(square(), {1, 0, 1, 1}), K::Planar)); // not positive
    CHECK_THROWS(SizingMesh(with_field(square(), {1, 1, 1, 1}), K::Volume)); // no tets
    MeshData bad = with_field(square(), {1, 1, 1, 1});
    bad.cells[CellType::Triangle].connectivity(1, 2) = 7;
    CHECK_THROWS(SizingMesh(bad, K::Planar)); // a missing vertex
}
