#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {

template <typename M>
long connected(
    const M& m,
    wmtk::Simplex i,
    wmtk::Simplex j,
    std::function<wmtk::Simplex(const wmtk::Tuple&)> extractor,
    wmtk::PrimitiveType type);

template <typename M>
long edge_connected(const M& m, wmtk::Simplex i, wmtk::Simplex j);

template <typename M>
long vertex_connected(const M& m, wmtk::Simplex i, wmtk::Simplex j);

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

wmtk::TriMesh topology_separate_2d(wmtk::TriMesh m);
} // namespace wmtk::components::internal