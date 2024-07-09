#include "EdgeMesh_examples.hpp"


namespace wmtk::tests {
EdgeMesh single_line()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(1, 2);

    edges.row(0) << 0, 1;

    m.initialize(edges);
    return m;
}

EdgeMesh two_segments()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(2, 2);

    edges.row(0) << 0, 1;
    edges.row(1) << 1, 2;

    m.initialize(edges);
    return m;
}

EdgeMesh multiple_lines(int64_t n)
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(n, 2);

    for(int j = 0; j < n; ++j) {
        edges.row(j) = Vector2l(j,j+1);
    }

    m.initialize(edges);
    return m;
}

EdgeMesh loop_lines()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(6, 2);

    edges.row(0) << 0, 1;
    edges.row(1) << 1, 2;
    edges.row(2) << 2, 3;
    edges.row(3) << 3, 4;
    edges.row(4) << 4, 5;
    edges.row(5) << 5, 0;

    m.initialize(edges);
    return m;
}

EdgeMesh self_loop()
{
    EdgeMesh m;
    RowVectors2l edges;

    edges.resize(1, 2);
    edges.row(0) << 0, 0;

    m.initialize(edges);
    return m;
}

EdgeMesh two_line_loop()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(2, 2);

    edges.row(0) << 0, 1;
    edges.row(1) << 1, 0;

    m.initialize(edges);
    return m;
}

} // namespace wmtk::tests
