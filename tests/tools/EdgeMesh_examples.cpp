#include "EdgeMesh_examples.hpp"


namespace wmtk::tests {
EdgeMesh simple_line()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(4, 2);
    edges.row(0) << 0, 1;
    edges.row(1) << 1, 2;
    edges.row(2) << 2, 3;
    edges.row(3) << 3, 4;
    m.initialize(edges);
    return m;
}
EdgeMesh loop_line()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(4, 2);
    edges.row(0) << 0, 1;
    edges.row(1) << 1, 2;
    edges.row(2) << 2, 3;
    edges.row(3) << 3, 0;
    m.initialize(edges);
    return m;
}
EdgeMesh multiple_lines()
{
    EdgeMesh m;
    RowVectors2l edges;
    edges.resize(8, 2);
    edges.row(0) << 0, 1;
    edges.row(1) << 1, 2;
    edges.row(2) << 2, 3;
    edges.row(3) << 3, 4;
    edges.row(4) << 5, 6;
    edges.row(5) << 6, 7;
    edges.row(6) << 7, 8;
    edges.row(7) << 8, 9;
    m.initialize(edges);
    return m;
}
} // namespace wmtk::tests