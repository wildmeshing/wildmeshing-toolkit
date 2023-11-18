#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

long edge_connected(const wmtk::TriMesh& m, Simplex i, Simplex j);

long vertex_connected(const wmtk::TriMesh& m, Simplex i, Simplex j);

long find_edge_index(const wmtk::TriMesh& m, wmtk::Tuple t);

long find_vertex_index(const wmtk::TriMesh& m, wmtk::Tuple t);

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m);
} // namespace wmtk::components::internal