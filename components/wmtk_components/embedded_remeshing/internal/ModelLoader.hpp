#pragma once
#include <Eigen/Core>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

namespace wmtk::components::internal {
void load_matrix_in_trimesh();
void tet_divide_a(RowVectors4l& T, long x, long y, long id0, long Toffset);
void tet_divide_b(RowVectors4l& T, long x, long y, long id0, long Toffset);
void load_matrix_in_tetmesh();
} // namespace wmtk::components::internal
