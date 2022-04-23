#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <wmtk/TetMesh.h>

#include <Tracy.hpp>
#include <limits>

namespace wmtk {
class ConcurrentTetMesh : public TetMesh
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
            mutex.unlock();
            reset_owner();
        }

        int get_owner() { return owner; }

        void set_owner(int n) { owner = n; }

        void reset_owner() { owner = INT_MAX; }
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

    void unlock_vertex_mutex(const Tuple& v)
    {
        
        m_vertex_mutex[v.vid(*this)].unlock();
    }
    void unlock_vertex_mutex(size_t vid)
    {
        
        m_vertex_mutex[vid].unlock();
    }

protected:
    void resize_vertex_mutex(size_t v) override
    {
        
        m_vertex_mutex.grow_to_at_least(v);
    }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;
    tbb::enumerable_thread_specific<std::vector<size_t>> get_one_ring_cache;

    ConcurrentTetMesh() = default;
    virtual ~ConcurrentTetMesh() = default;

    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);
    int release_vertex_mutex_in_stack();

    // helpers
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    bool try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid);
    bool try_set_vertex_mutex_two_ring_vid(size_t v, int threadid);

    // can be called
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid = 0);
    bool try_set_face_mutex_two_ring(const Tuple& f, int threadid = 0);
    bool try_set_face_mutex_two_ring(
        const Tuple& v1,
        const Tuple& v2,
        const Tuple& v3,
        int threadid = 0);
    bool try_set_face_mutex_two_ring(size_t v1, size_t v2, size_t v3, int threadid = 0);
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid = 0);

public:
    void for_each_edge(const std::function<void(const TetMesh::Tuple&)>&) override;
    void for_each_vertex(const std::function<void(const TetMesh::Tuple&)>&) override;
    void for_each_tetra(const std::function<void(const TetMesh::Tuple&)>&) override;
    int NUM_THREADS = 1;
};
} // namespace wmtk
