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

TriMesh single_equilateral_triangle(int dimension = 3);
TriMesh single_2d_nonequilateral_triangle_with_positions();

// a single triangle with position
TriMesh single_2d_triangle_with_random_positions(size_t seed = 123);
//  3--1--- 0
//   |     / \ .
//   2 f1 /2   1
//   |  0/ f0  \ .
//   |  /       \ .
//  1  ----0---- 2
//
TriMesh one_ear(); // an alias for quad
TriMesh quad();


//(0,1)    (1,1)
// 3 _______ 0
// |      /  |
// |     /   |
// |    /    |
// |   /     |
// |  /      |
// 1 ------- 2
// (0,0)    (1,0)
TriMesh unit_squre();

//  3--1--- 0 --1- 4
//   |     / \     |
//   2 f1 /2 1\ f2 |
//   |  0/ f0  \1  0
//   |  /       \  |
//   1  ----0----  2
//
TriMesh two_neighbors();


//   3 ----------- 4
//   |  \        / |
//   |   \  f3  /  |
//   |    \    /   |
//   |     \  /    |
//   |      0      |
//   |     / \     |
//   | f1 /   \ f2 |
//   |   / f0  \   |
//   |  /       \  |
//   1  ---------  2
//
TriMesh two_neighbors_plus_one();

//  3--1--- 6
//   |     /
//   2 f1 0
//   |   /
//   |  /  ^
//   5     |
//         |   0 --1- 4
//         v  / \     |
//           /2 1\ f2 |
//         0/ f0  \1  0
//         /       \  |
//      1  ----0----  2
TriMesh two_neighbors_cut_on_edge01();

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

TriMesh tetrahedron_with_position();

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

TriMesh hex_plus_two_with_position();

//    0---1---2
//   / \ / \ / \ .
//  3---4---5---6
//   \ / \ / \ /
//    7---8---9
TriMesh edge_region();

//  0---2  3---4
//  | /    | \ |
//  1      5---6
TriMesh three_triangles_with_two_components();


// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  0⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠞⠁⠀⡇⠉⠓⠦⢤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠞⠁⠀⠀⠀⠸⡄⠀⠀⠀⠀⠉⠓⢦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠋⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠀⠀⠀⠀⠀⠈⠓⠦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀
// ⠀⠀⠀⠀⠀⣠⠴⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⠦⢄⡀⠀⠀⠀⠀⠀
// ⠀⠀⣀⠔⠋⠁⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⠦⣄⠀⠀
// ⢀⡞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓
//  1⠹⡌⠉⠉⠉⠓⠒⠒⠒⠒⠒⠒⠦⠤⠤⠤⠤2⣤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠖⠒⠒3⠀
// ⠀⡇⠀⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠋⡏⡁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸
// ⠀⡇⠀⠀⠱⡄⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⢀⡞⣱⠇⣶⣷⡤⡀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⡜⠁⢸
// ⢠⠇⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⣋⣴⣟⣽⠥⠹⣯⣧⡆⠀⠀⠀⠀⠀⠀⠀⠀⢀⡖⠁⠀⢸
// ⢸⠀⠀⠀⠀⠀⠈⢦⠀⠀⠀⠀⠀⠀⣠⡟⣩⡟⢿⢸⡇⢀⣔⣯⣿⡆⠀⠀⠀⠀⠀⠀⠀⡴⠃⠀⠀⠀⢸
// ⠀⡇⠀⠀⠀⠀⠀⠀⠱⡄⠀⠀⣠⢿⣷⣿⣃⣀⠈⠙⠛⡿⠛⠋⢠⡿⡇⠀⠀⠀⠀⡴⠋⠀⠀⠀⠀⠀⡇
// ⠀⡇⠀⠀ ⠀⠀⠀⠀⠈⢇⡞⠁⠟⠓⠒⠛⠘⠙⠑⠂⠓⣒⣶⣿⡟⣆⣀⠀⢀⡏⠀⠀⠀⠀⠀⠀⣸⠀
// ⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀4⢯⠉⠉⠑⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠤⠤⠤⠤5⠀⠀⠀⠀⠀⠀⠀⡏⠀
// ⠀⡇⠀⠀⠀⠀⠀⠀⣠⠎⠀⠀⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠖⠁⠑⢆⠀⠀⠀⠀ ⢸
// ⠀⡇⠀⠀⠀⢀⠔⠃⠀⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠋⠀⠀⠀⠀⠀⠳⡄⠀⠀⢸
// ⠀⡇⢀⠖⠋⠀⠀⠀⠀⠀⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⣠⠔⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠙⡆⠀⡼
// ⠀6⣁⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡄⠀⠀⠀⣀⣠⠔⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣆⡇
// ⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠙⠿⠋⠛7⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠢⠤⠼⢦⡇⠀8⠀
TriMesh nine_triangles_with_a_hole();

TriMesh ten_triangles_with_position(int dimension);

TriMesh edge_region_with_position();
//      0---1
//     / \ / \ .
//    2---3---4
//   / \ / \ / \ .
//  5---6---7---8
//   \ / \ / \ /
//    9--10--11
//     \ / \ /
//     12---13
TriMesh embedded_diamond();

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


// 3    7____5 10 13__12
// |\   \f3/ | |\  \f5|
// |f1\2 \/f2| | \  \ |
// | / \  6\ | |  \  \|
// |/ f0\   \| |f4 \  11
// 0---- 1   4 8----9
//  16____15   19____18
//   \     |   |     /
//    \ f6 |   | f7 /
//     \   |   |   /
//      \  |   |  /
//       14    17
TriMesh cutup_uv_mesh_with_position();


//  3_____ 4____6
// | \f3 / |\    |
// |  \ /  | \ f5|
// |f1 2 f2|  \  |
// |  / \  |f4 \ |
// | /f0 \ |    \|
//  0----- 1 ----5
//   \     |    /
//    \ f6 | f7/
//     \   |  /
//      \  | /
//       \ |/
//         7
TriMesh sewed_at_seam_position_mesh_with_position();
// NOTE: in the future please create shared_ptr of meshes

//    6---1
//   / \ / \ .
//  5---0---2
//   \ / \ /  .
//    4---3
// creates N triangles surrounding a single interior vertex 0
std::shared_ptr<TriMesh> disk(int number);

// N triangles of
std::shared_ptr<TriMesh> individual_triangles(int number);


// creates N triangles surrounding a single interior vertex 0
std::shared_ptr<TriMesh> disk_to_individual_multimesh(int number);

std::shared_ptr<TriMesh> grid(int num_rows, bool set_double_);

} // namespace wmtk::tests
