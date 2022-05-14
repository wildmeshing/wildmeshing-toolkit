#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>
#include <wmtk/TriMesh.h>
#include <limits>


namespace wmtk {
class ConcurrentTriMesh : public TriMesh
{
public:
    class VertexMutex
    {
        tbb::spin_mutex mutex;
        int owner = std::numeric_limits<int>::max();

    public:
        bool trylock() { return mutex.try_lock(); }

        void unlock()
        {
            reset_owner();
            mutex.unlock();
        }

        int get_owner() { return owner; }

        void set_owner(int n) { owner = n; }

        void reset_owner() { owner = std::numeric_limits<int>::max(); }
    };

private:
    tbb::concurrent_vector<VertexMutex> m_vertex_mutex;

    bool try_set_vertex_mutex(const Tuple& v, int threadid)
    {
        bool got = m_vertex_mutex[v.vid(*this)].trylock();
        if (got) m_vertex_mutex[v.vid(*this)].set_owner(threadid);
        return got;
    }
    bool try_set_vertex_mutex(size_t vid, int threadid)
    {
        bool got = m_vertex_mutex[vid].trylock();
        if (got) m_vertex_mutex[vid].set_owner(threadid);
        return got;
    }

    void unlock_vertex_mutex(const Tuple& v) { m_vertex_mutex[v.vid(*this)].unlock(); }
    void unlock_vertex_mutex(size_t vid) { m_vertex_mutex[vid].unlock(); }

protected:
    void resize_mutex(size_t v) override { m_vertex_mutex.grow_to_at_least(v); }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;

    ConcurrentTriMesh() = default;
    virtual ~ConcurrentTriMesh() = default;
    // TODO remove later
    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);
    int release_vertex_mutex_in_stack();
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid);
};
} // namespace wmtk
