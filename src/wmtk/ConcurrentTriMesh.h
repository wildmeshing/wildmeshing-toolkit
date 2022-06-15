#pragma once

#include <wmtk/TriMesh.h>
#include <limits>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk {
/**
 * @brief child of TriMesh for running with multithread
 *
 */
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
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the two ends of an
     * edge
     *
     * @param e Tuple refers to the edge
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid);
    /**
     * @brief get the lock for one ring neighboring triangles' incident vertices
     *
     * @param v
     * @param threadid
     * @return true if all succeed
     */
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid);

    /**
     * @brief perform the given function for each face
     *
     */
    void for_each_face(const std::function<void(const Tuple&)>&);
    /**
     * @brief perform the given function for each edge
     *
     */
    void for_each_edge(const std::function<void(const Tuple&)>&);
    /**
     * @brief perform the given function for each vertex
     *
     */
    void for_each_vertex(const std::function<void(const Tuple&)>&);
    int NUM_THREADS = 1;
};
} // namespace wmtk
