#pragma once

#include <wmtk/TetMesh.hpp>

namespace wmtk::tests_3d {

//        0
//       / \\ .
//      /   \ \ .
//     /     \  \ .
//    /       \   \ 3
//  1 --------- 2
//

TetMesh single_tet();

//        0 ---------- 4
//       / \\        /
//      /   \ \     /
//     /     \  \  /
//    /       \  \\ 3
//  1 --------- 2/
//

TetMesh one_ear();

//  5 --------- 0 ---------- 4
//   \  \      / \\        /
//    \      \/   \ \     /
//     \     /    \\  \  /
//      \   /       \  \\ 3
//        1 --------- 2/      tuple edge 1-2
//

TetMesh two_ears();

//        0 ---------- 4
//       / \\        // \ .
//      /   \ \     //   \ .
//     /     \  \  //     \  .
//    /       \   \3       \ .
//  1 --------- 2/ -------- 5   tuple edge 2-3
//

TetMesh three_incident_tets();

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

TetMesh six_cycle_tets();


} // namespace wmtk::tests_3d
