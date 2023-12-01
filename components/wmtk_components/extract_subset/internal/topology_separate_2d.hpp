#pragma once

#include <functional>
#include <numeric>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "utils.hpp"

namespace wmtk::components::internal {

template <typename Extractor>
long connected(
    const wmtk::TriMesh& m,
    wmtk::Simplex i,
    wmtk::Simplex j,
    Extractor extractor,
    wmtk::PrimitiveType type);

long edge_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j);

long vertex_connected(const wmtk::TriMesh& m, wmtk::Simplex i, wmtk::Simplex j);

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