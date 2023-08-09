#include "TriMesh_examples.hpp"


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
    //   / \ / \ / \ .
    //  3---4---5---6
    //   \ / \ /  .
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

TriMesh strip(long size)
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
            t = Vector<long, 3>(j, j + 1, j + 2);
        } else {
            t = Vector<long, 3>(j, j + 2, j + 1);
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

} // namespace wmtk::tests
