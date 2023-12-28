#include "TetMesh_examples.hpp"

#include <wmtk/utils/mesh_utils.hpp>

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

TetMesh three_incident_tets_with_positions()
{
    TetMesh m = three_incident_tets();
    Eigen::Matrix<double, 6, 3> V;
    V.row(0) = Eigen::Vector3d(-4., 0., 0);
    V.row(1) = Eigen::Vector3d(2., 2., 0);
    V.row(2) = Eigen::Vector3d(0., 2., 0);
    V.row(3) = Eigen::Vector3d(0., 1, 1);
    V.row(4) = Eigen::Vector3d(0., 1, -1);
    V.row(5) = Eigen::Vector3d(1., 1, -1);

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
    return m;
}

TetMesh six_cycle_tets()
{
    //        0 ---------- 4
    //       / \\        // \ .
    //      /   \ \     //   \ .
    //     /     \  \  //     \ .
    //    /       \   \3       \ .
    //  1 --------- 2/ -------- 5   tuple edge 2-3
    //    \       /  /\ \      / .
    //     \     / /   \\     / .
    //      \   //      \\   / .
    //       \ //        \  / .
    //        6 -----------7
    //

    TetMesh m;
    RowVectors4l tets;
    tets.resize(6, 4);
    tets.row(0) << 0, 1, 2, 3;
    tets.row(1) << 0, 2, 3, 4;
    tets.row(2) << 2, 5, 3, 4;
    tets.row(3) << 6, 1, 2, 3;
    tets.row(4) << 6, 2, 3, 7;
    tets.row(5) << 7, 2, 3, 5;
    m.initialize(tets);
    return m;
}

TetMesh three_cycle_tets()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(3, 4);
    tets.row(0) << 0, 1, 2, 4;
    tets.row(1) << 0, 1, 4, 3;
    tets.row(2) << 0, 1, 3, 2;
    m.initialize(tets);
    return m;
}

TetMesh four_cycle_tets()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(4, 4);
    tets.row(0) << 0, 1, 3, 4;
    tets.row(1) << 0, 1, 4, 5;
    tets.row(2) << 0, 1, 5, 2;
    tets.row(3) << 0, 1, 2, 3;
    m.initialize(tets);
    return m;
}


} // namespace wmtk::tests_3d