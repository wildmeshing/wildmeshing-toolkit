#include "TetMesh_examples.hpp"


namespace wmtk::tests_3d {

TetMesh single_tet()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(1, 4);
    tets.row(0) << 0, 1, 2, 3;
    m.initialize(tets);
    return m;
}

TetMesh one_ear()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(2, 4);
    tets.row(0) << 0, 1, 2, 3;
    tets.row(1) << 0, 2, 3, 4;
    m.initialize(tets);
    return m;
}

TetMesh two_ears()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(3, 4);
    tets.row(0) << 0, 1, 2, 3;
    tets.row(1) << 0, 2, 3, 4;
    tets.row(2) << 0, 1, 3, 5;
    m.initialize(tets);
    return m;
}

TetMesh three_incident_tets()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(3, 4);
    tets.row(0) << 0, 1, 2, 3;
    tets.row(1) << 0, 2, 3, 4;
    tets.row(2) << 2, 5, 3, 4;
    m.initialize(tets);
    return m;
}


} // namespace wmtk::tests_3d