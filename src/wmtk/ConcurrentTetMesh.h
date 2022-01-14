#pragma once

#include <wmtk/TetMesh.h>
#include <tbb/concurrent_vector.h>

namespace wmtk {
class ConcurrentTetMesh : public TetMesh
{
private:
    tbb::concurrent_vector<VertexConnectivity> m_vertex_connectivity;
    tbb::concurrent_vector<TetrahedronConnectivity> m_tet_connectivity;

    int find_next_empty_slot_t();
    int find_next_empty_slot_v();
    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);
};
} // namespace wmtk
