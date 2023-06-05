#pragma once

#include "AdaptiveTessellation.h"
namespace adaptive_tessellation {

private:
wmtk::VertexLinksData seamed_vertex_link(const AdaptiveTessellation& m, const Tuple& vertex);
std::vector<size_t> seamed_edge_link(const AdaptiveTessellation& m, const Tuple& edge);

bool seamed_link_condition(const AdaptiveTessellation& m, const Tuple& edge);
} // namespace adaptive_tessellation
