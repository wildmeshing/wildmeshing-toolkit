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
TetMesh three_incident_tets_with_positions();

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

TetMesh six_cycle_tets_with_positions();
//
//                   2
//               /  / |
//           /    //  |
//       /       //    |
//    /        /_1     |
// 3 -------- 0-  \    |
//    ---__      \  \   |
//          --__    \ \ |
//               ---__\\ .
//                      4

TetMesh three_cycle_tets();

//   2 ---------- 3
//   |\\        //|
//   | \ \     // |
//   |  \  \  //  |
//   |   \   \1   |
//   |     0/     |
//   |   /  /\ \  |
//   |  / /   \\  |
//   | //      \\ |
//   |//        \ |
//   5 ---------- 4
//


TetMesh four_cycle_tets();

//               _ _  _ _  _ _
//             /_ _ /_ _ /_ _ /|
//   layer2-> /_ _ /_ _ /_ _ /||
//            | _/ |\_  |  _/||/
//   layer1-> |/_ _|_ _\|/_ _|/
//   look from up to bottom
//   layer 1
//   8 9 10 11
//   4 5 6 7
//   0 1 2 3
//   layer 2
//   20 21 22 23
//   16 17 18 19
//   12 13 14 15
TetMesh two_by_three_grids_tets();

//               _ _  _ _
//             /_ _ /_ _ /|
//   layer3-> /_ _ /_ _ /||
//            | _/ |\_  ||/
//   layer2-> |/_ _|_ _\|/|
//            |\_  |  _/||/
//   layer1-> |_ _\|/_ _|/
//
//   look from up to bottom
//   layer 1
//   6 7 8
//   3 4 5
//   0 1 2
//   layer 2
//   15 16 17
//   12 13 14
//   9 10 11
//   layer 3
//   24 25 26
//   21 22 23
//   18 19 20
TetMesh two_by_two_by_two_grids_tets();

} // namespace wmtk::tests_3d
