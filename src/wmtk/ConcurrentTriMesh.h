#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>
#include <wmtk/TriMesh.h>

namespace wmtk {
class ConcurrentTriMesh : public TriMesh
{
public:
    class VertexMutex
    {
        tbb::spin_mutex mutex;

    public:
        bool trylock() { return mutex.try_lock(); }

        void unlock() { mutex.unlock(); }
    };

private:
    tbb::concurrent_vector<VertexMutex> m_vertex_mutex;

    bool try_set_vertex_mutex(const Tuple& v) { return m_vertex_mutex[v.vid()].trylock(); }
    bool try_set_vertex_mutex(size_t vid) { return m_vertex_mutex[vid].trylock(); }

    void unlock_vertex_mutex(const Tuple& v) { m_vertex_mutex[v.vid()].unlock(); }
    void unlock_vertex_mutex(size_t vid) { m_vertex_mutex[vid].unlock(); }

protected:
    void resize_vertex_attributes(size_t v) override { m_vertex_mutex.grow_to_at_least(v); }

public:
    ConcurrentTriMesh() = default;
    virtual ~ConcurrentTriMesh() = default;
    // TODO remove later
    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);
    int release_vertex_mutex_in_stack(std::vector<size_t>& mutex_release_stack);
    bool try_set_vertex_mutex_two_ring(const Tuple& v, std::vector<size_t>& mutex_release_stack);
    bool try_set_edge_mutex_two_ring(const Tuple& e, std::vector<size_t>& mutex_release_stack);
};
} // namespace wmtk
