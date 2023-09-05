#pragma once

#include <wmtk/TetMesh.hpp>

namespace wmtk::tests_3d {

//        1
//       / \\
//      /   \ \
//     /     \  \
//    /       \   \ 4
//  2 --------- 3
//

TetMesh single_tet();

//        1 ---------- 5
//       / \\        /
//      /   \ \     /
//     /     \  \  /
//    /       \  \\ 4
//  2 --------- 3/
//

TetMesh one_ear();

//  6 --------- 1 ---------- 5
//   \  \      / \\        /
//    \      \/   \ \     /
//     \     /    \\  \  /
//      \   /       \  \\ 4
//        2 --------- 3/      tuple edge 2-3
//

TetMesh two_ears();

//        1 ---------- 5
//       / \\        // \
//      /   \ \     //   \
//     /     \  \  //     \
//    /       \   \4       \
//  2 --------- 3/ -------- 6   tuple edge 3-4
//

TetMesh three_incident_tets();


} // namespace wmtk::tests_3d