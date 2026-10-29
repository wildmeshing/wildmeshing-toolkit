#pragma once

#include <wmtk/Types.hpp>

#include <vector>

namespace wmtk::io {

/**
 * @brief What reading a mesh and cleaning it changed: what a history needs to start from the
 * files as they hold the mesh (wmtk/history/InputHistory.hpp), for a triangle mesh and for an
 * edge mesh alike.
 *
 * The vertices and the cells as given -- the triangles of a triangle mesh, the segments of an
 * edge mesh (a polyline's consecutive vertex pairs, in order), several inputs concatenated -- the
 * vertex of the result each vertex became, -1 for one that went, and the cell of the result each
 * cell became: cells keep their order, a duplicate segment becomes the one it duplicates, and a
 * cell left with a repeated vertex goes (-1). The cleaning fills it (clean_triangle_mesh,
 * clean_edge_mesh), whether the mesh came from a file or from memory.
 */
struct ReadProvenance
{
    MatrixXd V;
    MatrixXi cells;
    std::vector<int> vertex_map;
    std::vector<int> cell_map;
};

} // namespace wmtk::io
