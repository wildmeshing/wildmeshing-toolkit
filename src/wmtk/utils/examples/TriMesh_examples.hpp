#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::utils::examples::tri {

struct TriMeshVF
{
    MatrixXd V;
    MatrixXi F;
};

/**
 * @brief An equilateral triangle
 *
 *      2
 *     / \
 *    /   \
 *   / f0  \
 *  /       \
 * 0---------1
 *
 */
TriMeshVF single_triangle(int8_t dimension = 2);

/**
 * @brief Two triangles.
 *
 * f0: equilateral
 * f1: rectangular in v3
 *
 * 3-----2
 * |    / \
 * | f1/   \
 * |  / f0  \
 * | /       \
 * 0 --------- 1
 */
TriMeshVF two_triangles(int8_t dimension = 2);

/**
 * @brief One edge surrounded by triangles.
 *
 *   0---1---2
 *  / \ / \ / \
 * 3---4---5---6
 *  \ / \ / \ /
 *   7---8---9
 */
TriMeshVF edge_region(int8_t dimension = 2);

// TriMesh single_2d_nonequilateral_triangle_with_positions();
//
//// a single triangle with position
// TriMesh single_2d_triangle_with_random_positions(size_t seed = 123);
////  3--1--- 0
////   |     / \ .
////   2 f1 /2   1
////   |  0/ f0  \ .
////   |  /       \ .
////  1  ----0---- 2
////
//TriMesh one_ear(); // an alias for quad
// TriMesh quad();
//
//// global indices:
////  3--2--- 0 --3- 4
////   |     / \     |
////   5 f1 0   1 f2 |
////   |   / f0  \   6
////   |  /       \  |
////   1  ----4----  2
////
//// local indices:
////  0x-----2x0---- x2
////   |  1  /0\  1  |
////   |2   /2 1\   0|
////   |  0/     \2  |
////   |  /1  0  2\  |
////  1x  ---------  x1
////
// TriMesh two_neighbors();
//
//
////   3 ----------- 4
////   |  \        / |
////   |   \  f3  /  |
////   |    \    /   |
////   |     \  /    |
////   |      0      |
////   |     / \     |
////   | f1 /   \ f2 |
////   |   / f0  \   |
////   |  /       \  |
////   1  ---------  2
////
// TriMesh two_neighbors_plus_one();
//
////  3--1--- 6
////   |     /
////   2 f1 0
////   |   /
////   |  /  ^
////   5     |
////         |   0 --1- 4
////         v  / \     |
////           /2 1\ f2 |
////         0/ f0  \1  0
////         /       \  |
////      1  ----0----  2
// TriMesh two_neighbors_cut_on_edge01();
//
////
////  4------ 0 ---- 3
////   |     / \     |
////   | f2 /   \ f0 |
////   |   / f1  \   |
////   |  /       \  |
////   2  ---------  1
////      \       /  .
////       \ f3  /   .
////        \   /    .
////         \ /     .
////          5
// TriMesh three_neighbors();
//
////  3------ 0 ---- 3
////   |     / \     |
////   | f2 /   \ f0 |
////   |   / f1  \   |
////   |  /       \  |
////   2  ---------  1
////      \       /  .
////       \ f3  /   .
////        \   /    .
////         \ /     .
////          3
// TriMesh tetrahedron();
//
// TriMesh tetrahedron_with_position();
//
////  3--1--- 0
////   |     / \ .
////   2 f1 /2   1
////   |  0/ f0  \ .
////   |  /       \ .
////  1  ----0---- 2
////     \        /
////      \  f2  /
////       \    /
////        \  /
////         4
// TriMesh interior_edge();
//
//
////    .---.---.
////   /0\l/2\3/4\ .
////  .---.---.---.
////   \5/6\7/  .
////    .---.
////    0---1---2
////   / \ / \ / \ .
////  3---4---5---6
////   \ / \ /
////    7---8
////    .-0-.-3-.
////   1 2 4 5 6 7 .
////  .-8-.-a-.-d-.
////   9 b c e  .
////    .-f-.
// TriMesh hex_plus_two();
//
// TriMesh hex_plus_two_with_position();
//
////    0---1---2
////   / \ / \ / \ .
////  3---4---5---6
////   \ / \ / \ /
////    7---8---9
// TriMesh edge_region();
//
////  0---2  3---4
////  | /    | \ |
////  1      5---6
// TriMesh three_triangles_with_two_components();
//
//
//// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  0⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠞⠁⠀⡇⠉⠓⠦⢤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//// ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡠⠞⠁⠀⠀⠀⠸⡄⠀⠀⠀⠀⠉⠓⢦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//// ⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠋⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠀⠀⠀⠀⠀⠈⠓⠦⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//// ⠀⠀⠀⠀⠀⣠⠴⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠑⠦⢄⡀⠀⠀⠀⠀⠀
//// ⠀⠀⣀⠔⠋⠁⠀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⢧⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⠦⣄⠀⠀
//// ⢀⡞⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓
////  1⠹⡌⠉⠉⠉⠓⠒⠒⠒⠒⠒⠒⠦⠤⠤⠤⠤2⣤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠤⠖⠒⠒3⠀
//// ⠀⡇⠀⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠋⡏⡁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸
//// ⠀⡇⠀⠀⠱⡄⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⢀⡞⣱⠇⣶⣷⡤⡀⠀⠀⠀⠀ ⠀⠀⠀⠀⠀⠀⠀⡜⠁⢸
//// ⢠⠇⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⡴⣋⣴⣟⣽⠥⠹⣯⣧⡆⠀⠀⠀⠀⠀⠀⠀⠀⢀⡖⠁⠀⢸
//// ⢸⠀⠀⠀⠀⠀⠈⢦⠀⠀⠀⠀⠀⠀⣠⡟⣩⡟⢿⢸⡇⢀⣔⣯⣿⡆⠀⠀⠀⠀⠀⠀⠀⡴⠃⠀⠀⠀⢸
//// ⠀⡇⠀⠀⠀⠀⠀⠀⠱⡄⠀⠀⣠⢿⣷⣿⣃⣀⠈⠙⠛⡿⠛⠋⢠⡿⡇⠀⠀⠀⠀⡴⠋⠀⠀⠀⠀⠀⡇
//// ⠀⡇⠀⠀ ⠀⠀⠀⠀⠈⢇⡞⠁⠟⠓⠒⠛⠘⠙⠑⠂⠓⣒⣶⣿⡟⣆⣀⠀⢀⡏⠀⠀⠀⠀⠀⠀⣸⠀
//// ⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀4⢯⠉⠉⠑⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠤⠤⠤⠤5⠀⠀⠀⠀⠀⠀⠀⡏⠀
//// ⠀⡇⠀⠀⠀⠀⠀⠀⣠⠎⠀⠀⢧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠖⠁⠑⢆⠀⠀⠀⠀ ⢸
//// ⠀⡇⠀⠀⠀⢀⠔⠃⠀⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡤⠋⠀⠀⠀⠀⠀⠳⡄⠀⠀⢸
//// ⠀⡇⢀⠖⠋⠀⠀⠀⠀⠀⠀⠀⠀⠘⡄⠀⠀⠀⠀⠀⠀⠀⠀⣠⠔⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠙⡆⠀⡼
//// ⠀6⣁⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡄⠀⠀⠀⣀⣠⠔⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣆⡇
//// ⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠙⠿⠋⠛7⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠒⠢⠤⠼⢦⡇⠀8⠀
// TriMesh nine_triangles_with_a_hole();
//
// TriMesh ten_triangles_with_position(int dimension);
//
// TriMesh edge_region_with_position();
////      0---1
////     / \ / \ .
////    2---3---4
////   / \ / \ / \ .
////  5---6---7---8
////   \ / \ / \ /
////    9--10--11
////     \ / \ /
////     12---13
// TriMesh embedded_diamond();
//
////
////  4------ 0 ---- 3
////   |     / \     |
////   | f1 /   \ f0 |
////   |   /     \   |
////   |  /       \  |
////   2  ---------  1
////      \       /  .
////       \ f2  /   .
////        \   /    .
////         \ /     .
////          5
// TriMesh three_individuals();
//
//
//// NOTE: in the future please create shared_ptr of meshes
//
////    6---1
////   / \ / \ .
////  5---0---2
////   \ / \ /  .
////    4---3
//// creates N triangles surrounding a single interior vertex 0
// std::shared_ptr<TriMesh> disk(int number);
//
//// N triangles
// std::shared_ptr<TriMesh> individual_triangles(int number);
//
//
//// creates N triangles surrounding a single interior vertex 0
// std::shared_ptr<TriMesh> disk_to_individual_multimesh(int number);
//
// std::shared_ptr<TriMesh> grid(int num_rows, bool set_double_);

} // namespace wmtk::utils::examples::tri
