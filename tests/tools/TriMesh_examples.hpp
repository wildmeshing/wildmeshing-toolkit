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
TriMesh one_ear(); // an alias for quad
TriMesh quad();

//  3--1--- 0 --1- 4
//   |     / \     |
//   2 f1 /2 1\ f2 |
//   |  0/ f0  \1  0
//   |  /       \  |
//   1  ----0----  2
//
TriMesh two_neighbors();

//
//  4------ 0 ---- 3
//   |     / \     |
//   | f2 /   \ f0 |
//   |   / f1  \   |
//   |  /       \  |
//   2  ---------  1
//      \       /  .
//       \ f3  /   .
//        \   /    .
//         \ /     .
//          5
TriMesh three_neighbors();

//  3------ 0 ---- 3
//   |     / \     |
//   | f2 /   \ f0 |
//   |   / f1  \   |
//   |  /       \  |
//   2  ---------  1
//      \       /  .
//       \ f3  /   .
//        \   /    .
//         \ /     .
//          3
TriMesh tetrahedron();

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

//    0---1---2
//   / \ / \ / \ .
//  3---4---5---6
//   \ / \ / \ /
//    7---8---9
TriMesh edge_region();


//
//  4------ 0 ---- 3
//   |     / \     |
//   | f1 /   \ f0 |
//   |   /     \   |
//   |  /       \  |
//   2  ---------  1
//      \       /  .
//       \ f2  /   .
//        \   /    .
//         \ /     .
//          5
TriMesh three_individuals();
} // namespace wmtk::tests
