#include "TriMesh_examples.hpp"
#include <random>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk::tests {

TriMesh single_triangle()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) << 0, 1, 2;
    m.initialize(tris);
    return m;
}

TriMesh single_equilateral_triangle(int dimension)
{
    assert(dimension == 2 || dimension == 3);
    TriMesh m = single_triangle();
    Eigen::Matrix<double, 3, 3> V;

    V.row(0) << 0., 0., 0;
    V.row(1) << 1., 0, 0;
    V.row(2) << 0.5, sqrt(3) / 2., 0;

#if !defined(NDEBUG)
    auto xt = V.row(0);
    auto yt = V.row(1);
    auto zt = V.row(2);
    auto xth = xt.head<2>();
    auto yth = yt.head<2>();
    auto zth = zt.head<2>();
    auto x = xth.transpose();
    auto y = yth.transpose();
    auto z = zth.transpose();
    assert(wmtk::utils::triangle_signed_2d_area(x, y, z) >= 0);
#endif

    auto V2 = V.leftCols(dimension).eval();
    mesh_utils::set_matrix_attribute(V2, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh single_2d_triangle_with_random_positions(size_t seed)
{
    TriMesh m = single_triangle();
    Eigen::Matrix<double, 3, 2> V;

    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0., 1.);

    auto xt = V.row(0);
    auto yt = V.row(1);
    auto zt = V.row(2);

    auto x = xt.transpose();
    auto y = yt.transpose();
    auto z = zt.transpose();
    auto gen = [&](int, int) { return distribution(generator); };
    do {
        V = Eigen::MatrixXd::NullaryExpr(V.rows(), V.cols(), gen);
    } while (wmtk::utils::triangle_signed_2d_area(x, y, z) <= 0);


    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh single_2d_nonequilateral_triangle_with_positions()
{
    TriMesh m = single_triangle();
    Eigen::Matrix<double, 3, 2> V;
    V.row(0) = Eigen::Vector2d(-4., 0.);
    V.row(1) = Eigen::Vector2d(2., 2.);
    V.row(2) = Eigen::Vector2d(0., 2.);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}


TriMesh quad()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(2, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 1, 0;
    m.initialize(tris);
    return m;
}
TriMesh one_ear()
{
    return quad();
}

TriMesh two_neighbors()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 1, 0;
    tris.row(2) << 0, 2, 4;
    m.initialize(tris);
    return m;
}

TriMesh two_neighbors_plus_one()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(4, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 1, 0;
    tris.row(2) << 0, 2, 4;
    tris.row(3) << 3, 0, 4;
    m.initialize(tris);
    return m;
}


TriMesh two_neighbors_cut_on_edge01()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 5, 6;
    tris.row(2) << 0, 2, 4;
    m.initialize(tris);
    return m;
}

TriMesh three_neighbors()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(4, 3);
    tris.row(0) << 0, 3, 1;
    tris.row(1) << 0, 1, 2;
    tris.row(2) << 0, 2, 4;
    tris.row(3) << 2, 1, 5;
    m.initialize(tris);
    return m;
}

TriMesh tetrahedron()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(4, 3);
    tris.row(0) << 0, 3, 1;
    tris.row(1) << 0, 1, 2;
    tris.row(2) << 0, 2, 3;
    tris.row(3) << 2, 1, 3;
    m.initialize(tris);
    return m;
}

TriMesh tetrahedron_with_position()
{
    TriMesh m = tetrahedron();

    const double ost = 1.0 / std::sqrt(2.0);

    Eigen::MatrixXd V;
    V.resize(4, 3);
    V.row(0) << 1, 0, -ost;
    V.row(1) << -1, 0, -ost;
    V.row(2) << 0, 1, ost;
    V.row(3) << 0, -1, ost;
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh interior_edge()
{
    //  3--1--- 0
    //   |     / \ .
    //   2 f1 /2   1
    //   |  0/ f0  \ .
    //   |  /       \ .
    //  1  ----0---- 2
    //     \        /
    //      \  f2  /
    //       \    /
    //        \  /
    //         4
    TriMesh m;

    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 1, 0;
    tris.row(2) << 1, 4, 2;
    m.initialize(tris);
    return m;
}

TriMesh hex_plus_two()
{
    //    0---1---2
    //   /0\1/2\3/4\ .
    //  3---4---5---6
    //   \5/6\7/  .
    //    7---8
    TriMesh m;
    RowVectors3l tris;
    tris.resize(8, 3);
    tris.row(0) << 3, 4, 0;
    tris.row(1) << 4, 1, 0;
    tris.row(2) << 4, 5, 1;
    tris.row(3) << 5, 2, 1;
    tris.row(4) << 5, 6, 2;
    tris.row(5) << 3, 7, 4;
    tris.row(6) << 7, 8, 4;
    tris.row(7) << 4, 8, 5;
    m.initialize(tris);
    return m;
}

TriMesh hex_plus_two_with_position()
{
    TriMesh m = hex_plus_two();

    Eigen::MatrixXd V;
    V.resize(9, 3);
    V.row(0) << 0.5, 1, 0;
    V.row(1) << 1.5, 1, 0;
    V.row(2) << 2.5, 1, 0;
    V.row(3) << 0, 0, 0;
    V.row(4) << 1, 0, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 3, 0, 0;
    V.row(7) << 0.5, -1, 0;
    V.row(8) << 1.5, -1, 0;
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh edge_region()
{
    //    0---1---2
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ / \ /
    //    7---8---9
    TriMesh m;
    RowVectors3l tris;
    tris.resize(10, 3);
    tris.row(0) << 3, 4, 0;
    tris.row(1) << 4, 1, 0;
    tris.row(2) << 4, 5, 1;
    tris.row(3) << 5, 2, 1;
    tris.row(4) << 5, 6, 2;
    tris.row(5) << 3, 7, 4;
    tris.row(6) << 7, 8, 4;
    tris.row(7) << 4, 8, 5;
    tris.row(8) << 8, 9, 5;
    tris.row(9) << 5, 9, 6;
    m.initialize(tris);
    return m;
}

TriMesh edge_region_with_position()
{
    TriMesh m = edge_region();

    Eigen::MatrixXd V;
    V.resize(10, 3);
    V.row(0) << 0.5, 1, 0;
    V.row(1) << 1.5, 1, 0;
    V.row(2) << 2.5, 1, 0;
    V.row(3) << 0, 0, 0;
    V.row(4) << 1, 0, 0;
    V.row(5) << 2, 0, 0;
    V.row(6) << 3, 0, 0;
    V.row(7) << 0.5, -1, 0;
    V.row(8) << 1.5, -1, 0;
    V.row(9) << 2.5, -1, 0;
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh embedded_diamond()
{
    //      0---1
    //     / \ / \ .
    //    2---3---4
    //   / \ / \ / \ .
    //  5---6---7---8
    //   \ / \ / \ /
    //    9--10--11
    //     \ / \ /
    //     12---13
    TriMesh m;
    RowVectors3l tris;
    tris.resize(16, 3);
    tris.row(0) << 2, 3, 0;
    tris.row(1) << 3, 1, 0;
    tris.row(2) << 3, 4, 1;

    tris.row(3) << 5, 6, 2;
    tris.row(4) << 6, 3, 2;
    tris.row(5) << 6, 7, 3;
    tris.row(6) << 7, 4, 3;
    tris.row(7) << 7, 8, 4;

    tris.row(8) << 9, 6, 5;
    tris.row(9) << 9, 10, 6;
    tris.row(10) << 10, 7, 6;
    tris.row(11) << 10, 11, 7;
    tris.row(12) << 11, 8, 7;

    tris.row(13) << 12, 10, 9;
    tris.row(14) << 12, 13, 10;
    tris.row(15) << 13, 11, 10;
    m.initialize(tris);
    return m;
}

TriMesh strip(int64_t size)
{
    TriMesh m;

    //  0   1
    //  2   3
    //  4   5
    //  6   7

    RowVectors3l tris;
    tris.resize(size, 3);
    for (int j = 0; j < size; ++j) {
        auto t = tris.row(j);
        if (j % 2 == 0) {
            t = Vector<int64_t, 3>(j, j + 1, j + 2);
        } else {
            t = Vector<int64_t, 3>(j, j + 2, j + 1);
        }
    }
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 1, 0;
    tris.row(2) << 1, 4, 2;
    m.initialize(tris);
    return m;
}


TriMesh three_triangles_with_two_components()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 3, 6, 4;
    tris.row(2) << 3, 5, 6;
    m.initialize(tris);
    return m;
}

TriMesh nine_triangles_with_a_hole()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(9, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 0, 2, 3;
    tris.row(2) << 1, 4, 2;
    tris.row(3) << 1, 6, 4;
    tris.row(4) << 6, 7, 4;
    tris.row(5) << 4, 7, 5;
    tris.row(6) << 7, 8, 5;
    tris.row(7) << 5, 8, 3;
    tris.row(8) << 5, 3, 2;
    m.initialize(tris);
    return m;
}

TriMesh ten_triangles_with_position(int dimension)
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(10, 3);
    tris.row(0) << 0, 1, 2;
    tris.row(1) << 0, 2, 3;
    tris.row(2) << 1, 4, 2;
    tris.row(3) << 1, 6, 4;
    tris.row(4) << 6, 7, 4;
    tris.row(5) << 4, 7, 5;
    tris.row(6) << 7, 8, 5;
    tris.row(7) << 5, 8, 3;
    tris.row(8) << 5, 3, 2;
    tris.row(9) << 2, 4, 5;
    m.initialize(tris);

    Eigen::MatrixXd V;
    V.resize(9, 3);
    V.row(0) << 0, 1, 0;
    V.row(1) << -1, 0, 0;
    V.row(2) << 0, 0, 0;
    V.row(3) << 1, 0, 0;
    V.row(4) << -0.8, -0.3, 0;
    V.row(5) << 1, -1, 0;
    V.row(6) << -3, -3, 0;
    V.row(7) << 0, -3, 0;
    V.row(8) << 1.5, -2, 0;

    if (dimension != 2 && dimension != 3) assert(false);

    V.conservativeResize(9, dimension);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TriMesh three_individuals()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) << 0, 3, 1;
    tris.row(1) << 0, 2, 4;
    tris.row(2) << 2, 1, 5;
    m.initialize(tris);
    return m;
}

std::shared_ptr<TriMesh> disk(int number)
{
    assert(number >= 1);
    auto mptr = std::make_shared<TriMesh>();
    TriMesh& m = *mptr;
    RowVectors3l tris;
    tris.resize(number, 3);
    tris.rowwise() = Vector3l(0, 1, 2).transpose();
    auto mut = tris.rightCols<2>();
    for (int j = 0; j < number; ++j) {
        mut.row(j).array() += j;
    }

    tris(number - 1, 2) = 1;
    m.initialize(tris);
    return mptr;
}

// N triangles of
std::shared_ptr<TriMesh> individual_triangles(int number)
{
    assert(number >= 1);

    auto mptr = std::make_shared<TriMesh>();
    TriMesh& m = *mptr;
    RowVectors3l tris;
    tris.resize(number, 3);
    tris.rowwise() = Vector3l(0, 1, 2).transpose();
    for (int j = 0; j < number; ++j) {
        tris.row(j).array() += 3 * j;
    }
    m.initialize(tris);
    return mptr;
}

#if defined(WMTK_ENABLE_MULTIMESH)
std::shared_ptr<TriMesh> disk_to_individual_multimesh(int number)
{
    auto d = disk(number);
    auto i = individual_triangles(number);
    auto map = multimesh::same_simplex_dimension_bijection(*d, *i);

    d->register_child_mesh(i, map);
    return d;
}
#endif
} // namespace wmtk::tests
