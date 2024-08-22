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
TetMesh six_cycle_tets_with_positions()
{
    TetMesh m = six_cycle_tets();
    Eigen::Matrix<double, 8, 3> V;
    V.row(2) = Eigen::Vector3d(0., 0., -1);
    V.row(3) = Eigen::Vector3d(0., 0., 1);

    V.row(1) = Eigen::Vector3d(-1., 0., 0);
    V.row(5) = Eigen::Vector3d(1., 0., 0);

    V.row(0) = Eigen::Vector3d(1., -1, 0);
    V.row(7) = Eigen::Vector3d(-1., 1, 0);

    V.row(4) = Eigen::Vector3d(1., 1, 0);
    V.row(6) = Eigen::Vector3d(-1., -1, 0);

    V.array() += 5.0;

    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, m);
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

TetMesh two_by_three_grids_tets()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(30, 4);
    tets.row(0) << 0, 1, 5, 13;
    tets.row(1) << 0, 12, 13, 16;
    tets.row(2) << 0, 4, 16, 5;
    tets.row(3) << 13, 16, 17, 5;
    tets.row(4) << 0, 5, 13, 16;

    tets.row(5) << 1, 2, 5, 13;
    tets.row(6) << 13, 14, 18, 2;
    tets.row(7) << 13, 17, 18, 5;
    tets.row(8) << 2, 5, 6, 18;
    tets.row(9) << 2, 5, 13, 18;

    tets.row(10) << 2, 3, 7, 15;
    tets.row(11) << 2, 6, 7, 18;
    tets.row(12) << 2, 14, 15, 18;
    tets.row(13) << 7, 15, 19, 18;
    tets.row(14) << 2, 7, 15, 18;

    tets.row(15) << 4, 5, 8, 16;
    tets.row(16) << 20, 16, 21, 8;
    tets.row(17) << 16, 17, 21, 5;
    tets.row(18) << 5, 8, 9, 21;
    tets.row(19) << 5, 8, 16, 21;

    tets.row(20) << 5, 6, 10, 18;
    tets.row(21) << 5, 9, 10, 21;
    tets.row(22) << 10, 18, 21, 22;
    tets.row(23) << 5, 10, 18, 21;
    tets.row(24) << 5, 17, 18, 21;

    tets.row(25) << 6, 7, 10, 18;
    tets.row(26) << 7, 10, 11, 23;
    tets.row(27) << 10, 18, 22, 23;
    tets.row(28) << 7, 18, 19, 23;
    tets.row(29) << 7, 10, 18, 23;

    m.initialize(tets);
    return m;
}

TetMesh two_by_two_by_two_grids_tets()
{
    TetMesh m;
    RowVectors4l tets;
    tets.resize(40, 4);
    tets.row(0) << 0, 1, 3, 9;
    tets.row(1) << 1, 4, 3, 13;
    tets.row(2) << 9, 13, 12, 3;
    tets.row(3) << 9, 10, 13, 1;
    tets.row(4) << 9, 13, 1, 3;

    tets.row(5) << 1, 2, 5, 11;
    tets.row(6) << 1, 5, 4, 13;
    tets.row(7) << 10, 11, 13, 1;
    tets.row(8) << 11, 13, 14, 5;
    tets.row(9) << 1, 5, 11, 13;

    tets.row(10) << 3, 4, 13, 7;
    tets.row(11) << 3, 7, 6, 15;
    tets.row(12) << 12, 13, 15, 3;
    tets.row(13) << 13, 16, 15, 7;
    tets.row(14) << 13, 15, 3, 7;

    tets.row(15) << 4, 5, 7, 13;
    tets.row(16) << 5, 8, 7, 17;
    tets.row(17) << 13, 14, 17, 5;
    tets.row(18) << 13, 17, 16, 7;
    tets.row(19) << 13, 17, 5, 7;

    tets.row(20) << 9, 10, 13, 19;
    tets.row(21) << 9, 13, 12, 21;
    tets.row(22) << 18, 19, 21, 9;
    tets.row(23) << 19, 22, 21, 13;
    tets.row(24) << 19, 21, 9, 13;

    tets.row(25) << 10, 11, 13, 19;
    tets.row(26) << 11, 14, 13, 23;
    tets.row(27) << 19, 23, 22, 13;
    tets.row(28) << 19, 20, 23, 11;
    tets.row(29) << 19, 23, 11, 13;

    tets.row(30) << 12, 13, 15, 21;
    tets.row(31) << 13, 16, 15, 25;
    tets.row(32) << 21, 25, 24, 15;
    tets.row(33) << 21, 22, 25, 13;
    tets.row(34) << 21, 25, 13, 15;

    tets.row(35) << 13, 14, 17, 23;
    tets.row(36) << 13, 17, 16, 25;
    tets.row(37) << 22, 23, 25, 13;
    tets.row(38) << 23, 26, 25, 17;
    tets.row(39) << 23, 25, 13, 17;

    m.initialize(tets);
    return m;
}

} // namespace wmtk::tests_3d
