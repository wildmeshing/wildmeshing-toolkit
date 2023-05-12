#pragma once

#include <wmtk/TriMesh.h>
namespace TriMesh {

struct VertexLinksData
{
    std::vector<size_t> vertex_link;
    std::vector<std::pair<size_t, size_t>> edge_link;
};
VertexLinksData vertex_links(const TriMesh& m, const Tuple& vertex);
std::vector<size_t> edge_link(const TriMesh& m, const Tuple& edge);

bool link_condition(const TriMesh& m, const Tuple& edge);
} // namespace TriMesh
