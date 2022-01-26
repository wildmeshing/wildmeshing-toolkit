#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <wmtk/TetMesh.h>

#include <Tracy.hpp>

namespace wmtk {
class ConcurrentTetMesh : public TetMesh
{
public:
    class VertexMutex
    {
        tbb::spin_mutex mutex;
        int owner = INT_MAX;

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
        ZoneScoped;
        bool got = m_vertex_mutex[v.vid(*this)].trylock();
        if (got) m_vertex_mutex[v.vid(*this)].set_owner(threadid);
        return got;
    }
    bool try_set_vertex_mutex(size_t vid, int threadid)
    {
        ZoneScoped;
        bool got = m_vertex_mutex[vid].trylock();
        if (got) m_vertex_mutex[vid].set_owner(threadid);
        return got;
    }

    void unlock_vertex_mutex(const Tuple& v)
    {
        ZoneScoped;
        m_vertex_mutex[v.vid(*this)].unlock();
    }
    void unlock_vertex_mutex(size_t vid)
    {
        ZoneScoped;
        m_vertex_mutex[vid].unlock();
    }

protected:
    void resize_vertex_mutex(size_t v) override
    {
        ZoneScoped;
        m_vertex_mutex.grow_to_at_least(v);
    }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;

    ConcurrentTetMesh() = default;
    virtual ~ConcurrentTetMesh() = default;

    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);
    // int release_vertex_mutex_in_stack(std::vector<size_t>& mutex_release_stack);
    int release_vertex_mutex_in_stack();

    // helpers
    bool try_set_vertex_mutex_two_ring(
        const Tuple& v,
        // std::vector<size_t>& mutex_release_stack,
        int threadid);
    bool try_set_vertex_mutex_two_ring_vid(
        const Tuple& v,
        // std::vector<size_t>& mutex_release_stack,
        int threadid);
    bool try_set_vertex_mutex_two_ring_vid(
        size_t v,
        // std::vector<size_t>& mutex_release_stack,
        int threadid);

    // can be called
    bool try_set_edge_mutex_two_ring(
        const Tuple& e,
        // std::vector<size_t>& mutex_release_stack,
        int threadid = 0);
    bool try_set_face_mutex_two_ring(
        const Tuple& f,
        // std::vector<size_t>& mutex_release_stack,
        int threadid = 0);
    bool try_set_face_mutex_two_ring(
        const Tuple& v1,
        const Tuple& v2,
        const Tuple& v3,
        // std::vector<size_t>& mutex_release_stack,
        int threadid = 0);
    bool try_set_face_mutex_two_ring(
        size_t v1,
        size_t v2,
        size_t v3,
        // std::vector<size_t>& mutex_release_stack,
        int threadid = 0);
    bool try_set_vertex_mutex_one_ring(
        const Tuple& v,
        // std::vector<size_t>& mutex_release_stack,
        int threadid = 0);
};
} // namespace wmtk
