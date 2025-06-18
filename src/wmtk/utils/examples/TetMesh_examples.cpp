#include "TetMesh_examples.hpp"

namespace wmtk::utils::examples::tet {

TetMeshVT single_tet()
{
    TetMeshVT m;
    auto& V = m.V;
    auto& T = m.T;

    T.resize(1, 4);
    T.row(0) << 0, 1, 2, 3;

    V.resize(4, 3);
    V.row(0) << 0, 0, 0;
    V.row(1) << 0, 0, 1;
    V.row(2) << 0, 1, 0;
    V.row(3) << 1, 0, 0;

    return m;
}

TetMeshVT two_tets()
{
    TetMeshVT m;
    auto& V = m.V;
    auto& T = m.T;

    T.resize(2, 4);
    T.row(0) << 0, 1, 2, 3;
    T.row(1) << 0, 2, 3, 4;

    // TODO check if positions make sense
    V.resize(5, 3);
    V.row(0) << 0, 0, 0;
    V.row(1) << 0, 0, 1;
    V.row(2) << 0, 1, 0;
    V.row(3) << 1, 0, 0;
    V.row(4) << 1, 1, 1;

    return m;
}

// TetMeshVT two_ears()
// {
//     TetMesh m;
//     RowVectors4l tets;
//     tets.resize(3, 4);
//     tets.row(0) << 0, 1, 2, 3;
//     tets.row(1) << 0, 2, 3, 4;
//     tets.row(2) << 0, 1, 3, 5;
//     m.initialize(tets);
//     return m;
// }

TetMeshVT three_incident_tets()
{
    TetMeshVT m;
    auto& V = m.V;
    auto& T = m.T;

    T.resize(3, 4);
    T.row(0) << 0, 1, 2, 3;
    T.row(1) << 0, 2, 3, 4;
    T.row(2) << 2, 5, 3, 4;

    V.resize(6, 3);
    V.row(0) = Eigen::Vector3d(-4., 0., 0);
    V.row(1) = Eigen::Vector3d(2., 2., 0);
    V.row(2) = Eigen::Vector3d(0., 2., 0);
    V.row(3) = Eigen::Vector3d(0., 1, 1);
    V.row(4) = Eigen::Vector3d(0., 1, -1);
    V.row(5) = Eigen::Vector3d(1., 1, -1);

    return m;
}

TetMeshVT six_cycle_tets()
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

    TetMeshVT m;
    auto& V = m.V;
    auto& T = m.T;

    T.resize(6, 4);
    T.row(0) << 0, 1, 2, 3;
    T.row(1) << 0, 2, 3, 4;
    T.row(2) << 2, 5, 3, 4;
    T.row(3) << 6, 1, 2, 3;
    T.row(4) << 6, 2, 3, 7;
    T.row(5) << 7, 2, 3, 5;

    V.resize(8, 3);
    V.row(2) = Eigen::Vector3d(0, 0, -1);
    V.row(3) = Eigen::Vector3d(0, 0, 1);

    V.row(1) = Eigen::Vector3d(-1, 0, 0);
    V.row(5) = Eigen::Vector3d(1, 0, 0);

    V.row(0) = Eigen::Vector3d(1, -1, 0);
    V.row(7) = Eigen::Vector3d(-1, 1, 0);

    V.row(4) = Eigen::Vector3d(1, 1, 0);
    V.row(6) = Eigen::Vector3d(-1, -1, 0);

    return m;
}

// TetMeshVT three_cycle_tets()
// {
//     TetMesh m;
//     RowVectors4l tets;
//     tets.resize(3, 4);
//     tets.row(0) << 0, 1, 2, 4;
//     tets.row(1) << 0, 1, 4, 3;
//     tets.row(2) << 0, 1, 3, 2;
//     m.initialize(tets);
//     return m;
// }

// TetMeshVT four_cycle_tets()
// {
//     TetMesh m;
//     RowVectors4l tets;
//     tets.resize(4, 4);
//     tets.row(0) << 0, 1, 3, 4;
//     tets.row(1) << 0, 1, 4, 5;
//     tets.row(2) << 0, 1, 5, 2;
//     tets.row(3) << 0, 1, 2, 3;
//     m.initialize(tets);
//     return m;
// }

} // namespace wmtk::utils::examples::tet
