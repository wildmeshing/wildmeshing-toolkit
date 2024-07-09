#pragma once

#include <wmtk/EdgeMesh.hpp>

namespace wmtk::tests {

/*
   0 --- 1
*/
EdgeMesh single_line();

/*
   0 --- 1 --- 2
*/
EdgeMesh two_segments();

/*
   0 -- 1 -- 2 -- 3 -- 4 -- 5
*/
EdgeMesh multiple_lines(int64_t n = 5);

/*
   0 -- 1 -- 2 -- 3 -- 4 -- 5 -- 0*
*/

EdgeMesh loop_lines();

/*
    0 --- 0
*/
EdgeMesh self_loop();

/*
    0 -- 1 -- 0*
*/
EdgeMesh two_line_loop();

} // namespace wmtk::tests
