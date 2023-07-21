#include "TriMesh_examples.hpp"


namespace wmtk::tests {

TriMesh single_triangle()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(1, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    m.initialize(tris);
    return m;
}

TriMesh quad()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(2, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
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
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
    tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 2, 4};
    m.initialize(tris);
    return m;
}

TriMesh three_neighbors()
{
    TriMesh m;
    RowVectors3l tris;
    tris.resize(4, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 3, 1};
    tris.row(1) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris.row(2) = Eigen::Matrix<long, 3, 1>{0, 2, 4};
    tris.row(3) = Eigen::Matrix<long, 3, 1>{2, 1, 5};
    m.initialize(tris);
    return m;
}

TriMesh interior_edge()
{
    TriMesh m;

    RowVectors3l tris;
    tris.resize(3, 3);
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
    tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
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
    // tris << 3, 4, 0, 4, 1, 0, 4, 5, 1, 5, 2, 1, 5, 6, 2, 3, 7, 4, 7, 8, 4, 4, 8, 5;
    tris << 3, 4, 0, 4, 1, 0, 4, 5, 1, 5, 2, 1, 5, 6, 2, 4, 8, 5, 3, 7, 4, 7, 8, 4;
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
    tris.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris.row(1) = Eigen::Matrix<long, 3, 1>{3, 1, 0};
    tris.row(2) = Eigen::Matrix<long, 3, 1>{1, 4, 2};
    m.initialize(tris);
    return m;
}
} // namespace wmtk::tests

