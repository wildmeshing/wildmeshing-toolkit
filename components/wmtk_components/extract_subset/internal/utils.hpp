#pragma once

#include <functional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {

template <typename Extractor>
long connected(
    const wmtk::Mesh& m,
    wmtk::Simplex i,
    wmtk::Simplex j,
    Extractor extractor,
    wmtk::PrimitiveType type);

long face_connected(const wmtk::Mesh& m, wmtk::Simplex i, wmtk::Simplex j);

long edge_connected(const wmtk::Mesh& m, wmtk::Simplex i, wmtk::Simplex j);

long vertex_connected(const wmtk::Mesh& m, wmtk::Simplex i, wmtk::Simplex j);

long find_index(
    const wmtk::Mesh& m,
    wmtk::Tuple t,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractFunction,
    wmtk::PrimitiveType type);

long find_edge_index(const wmtk::Mesh& m, wmtk::Tuple t);

long find_vertex_index(const wmtk::Mesh& m, wmtk::Tuple t);

std::vector<long> adj_faces_of_vertex(const wmtk::TriMesh& m, long i);

void dfs(
    long start,
    std::vector<bool>& visited,
    std::vector<long>& cc,
    const std::vector<std::vector<long>>& adj,
    const std::function<bool(long, std::vector<long>&)>& condition,
    std::vector<long>& candidates);

std::vector<std::vector<long>> cc_around_vertex(
    const wmtk::TriMesh& m,
    std::vector<long>& adj_faces,
    std::vector<std::vector<long>>& adj_list_faces);

} // namespace wmtk::components::internal