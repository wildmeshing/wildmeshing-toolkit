#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk::tests {


//         0
//        / \   .
//       2   1  \ .
//      /  0  \  \|
//     /       \ .
//  1  ----0---- 2
//
TriMesh single_triangle();

//  3--1--- 0
//   |     / \ .
//   2 f1 /2   1
//   |  0/ f0  \ .
//   |  /       \ .
//  1  ----0---- 2
//
TriMesh one_ear();

//  3--1--- 0 --1- 4
//   |     / \     |
//   2 f1 /2 1\ f2 |
//   |  0/ f0  \1  0
//   |  /       \  |
//   1  ----0----  2
//
TriMesh two_ears();

//  3--1--- 0
//   |     / \ .
//   2 f1 /2   1
//   |  0/ f0  \ .
//   |  /       \ .
//  1  ----0---- 2
//     \        /
//      \  f2  /
//       \    /
//        \  /
//         4
TriMesh interior_edge();


//    0---1---2
//   / \ / \ / \ .
//  3---4---5---6
//   \ / \ /  .
//    7---8
TriMesh hex_plus_two();
} // namespace wmtk::tests
