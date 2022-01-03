
#pragma once

#include <wmtk/TriMesh.h>
#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>

namespace wmtk {
class ConcurrentTriMesh : public TriMesh
{
private:
    class VertexConnectivity
    {
    public:
        std::vector<size_t> m_conn_tris;
        bool m_is_removed = false;
        tbb::spin_mutex mutex;

        inline size_t& operator[](const size_t index)
        {
            assert(index >= 0 && index < m_conn_tris.size());
            return m_conn_tris[index];
        }

        inline size_t operator[](const size_t index) const
        {
            assert(index >= 0 && index < m_conn_tris.size());
            return m_conn_tris[index];
        }
    };

    tbb::concurrent_vector<VertexConnectivity> m_vertex_connectivity;
    tbb::concurrent_vector<TriangleConnectivity> m_tri_connectivity;

public:

    size_t get_next_empty_slot_t()
    {
        m_tri_connectivity.grow_by(1);
        return m_tri_connectivity.size() - 1;
    }
    size_t get_next_empty_slot_v()
    {
        m_vertex_connectivity.grow_by(1);
        return m_vertex_connectivity.size() - 1;
    }

    ConcurrentTriMesh() {}
    virtual ~ConcurrentTriMesh() {}

    inline void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
    {
        m_vertex_connectivity.grow_to_at_least(n_vertices);
        m_tri_connectivity.grow_to_at_least(tris.size());
        size_t hash_cnt = 0;
        for (int i = 0; i < tris.size(); i++) {
            m_tri_connectivity[i].m_indices = tris[i];
            m_tri_connectivity[i].hash = hash_cnt;
            hash_cnt++;
            for (int j = 0; j < 3; j++) m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
        }
    }

    void set_vertex_lock(VertexConnectivity &v){
        // TO DO
        // need to decide a strategy
        return;
    }

};
} // namespace wmtk

