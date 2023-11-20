#pragma once
#include <Eigen/Core>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {
void tri_divide_a(RowVectors3l& tris, long grid_x, long grid_y, long id0, long Toffset);
void tri_divide_b(RowVectors3l& tris, long grid_x, long grid_y, long id0, long Toffset);
void load_matrix_in_trimesh(TriMesh& mesh, const std::vector<std::vector<long>>& labels);
void tet_divide_a(RowVectors4l& tets, long grid_x, long grid_y, long id0, long Toffset);
void tet_divide_b(RowVectors4l& tets, long grid_x, long grid_y, long id0, long Toffset);
void load_matrix_in_tetmesh(
    TetMesh& mesh,
    const std::vector<std::vector<std::vector<long>>>& labels);
} // namespace wmtk::components::internal
