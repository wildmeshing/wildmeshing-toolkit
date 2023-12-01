#pragma once

#include <functional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

namespace wmtk::components::internal {

template <typename M>
long find_index(
    const M& m,
    wmtk::Tuple t,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractFunction,
    wmtk::PrimitiveType type);

template <typename M>
long find_edge_index(const M& m, wmtk::Tuple t);

template <typename M>
long find_vertex_index(const M& m, wmtk::Tuple t);

template <typename M>
long find_face_index(const M& m, wmtk::Tuple t);

template <typename M>
void get_edge_count(const M& m, std::vector<bool>& edge_count);

template <typename M>
bool vertex_on_boundary(const M& m, std::vector<bool>& edge_count, long i);

std::vector<long> adj_faces_of_vertex(const wmtk::TriMesh& m, long i);
} // namespace wmtk::components::internal