#pragma once

#include <functional>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

long find_index(
    const wmtk::TriMesh& m,
    wmtk::Tuple t,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractFunction,
    wmtk::PrimitiveType type);

long find_edge_index(const wmtk::TriMesh& m, wmtk::Tuple t);

long find_vertex_index(const wmtk::TriMesh& m, wmtk::Tuple t);

long find_face_index(const wmtk::TriMesh& m, wmtk::Tuple t);

void get_edge_count(const wmtk::TriMesh& m, std::vector<bool>& edge_count);

bool vertex_on_boundary(const wmtk::TriMesh& m, std::vector<bool>& edge_count, long i);

std::vector<long> adj_faces_of_vertex(const wmtk::TriMesh& m, long i);
} // namespace wmtk::components::internal