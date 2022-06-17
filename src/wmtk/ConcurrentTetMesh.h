#pragma once

#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/spin_mutex.h>
#include <wmtk/TetMesh.h>

#include <Tracy.hpp>
#include <limits>

namespace wmtk {
/**
 * @brief child of TetMesh for concurrent implementation
 *
 */
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

    void unlock_vertex_mutex(const Tuple& v) { m_vertex_mutex[v.vid(*this)].unlock(); }
    void unlock_vertex_mutex(size_t vid) { m_vertex_mutex[vid].unlock(); }

protected:
    void resize_vertex_mutex(size_t v) override { m_vertex_mutex.grow_to_at_least(v); }

public:
    tbb::enumerable_thread_specific<std::vector<size_t>> mutex_release_stack;
    tbb::enumerable_thread_specific<std::vector<size_t>> get_one_ring_cache;

    ConcurrentTetMesh() = default;
    virtual ~ConcurrentTetMesh() = default;

    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);
    int release_vertex_mutex_in_stack();

    // helpers
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring(const Tuple& v, int threadid);
    /**
     * @brief try lock the two-ring neighboring traingles' incident vertices using vids
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid);
    /**
     * @brief a duplicate of ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid that gets vids
     using the vid of the input Tuple
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_two_ring_vid(size_t v, int threadid);

    // can be called
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the two ends of an
     * edge
     *
     * @param e Tuple refers to the edge
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_edge_mutex_two_ring(const Tuple& e, int threadid = 0);
    /**
     * @brief try lock the two-ring neighboring triangles' incident vertices for the 3 vertices of a
     * face
     *
     * @param f Tuple refers to the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(const Tuple& f, int threadid = 0);
    /**
     * @brief locking the two-ring neighboring triangles' incident vertices given the 3 vertex
     * Tuples of the face
     *
     * @param v1 a Tuple refering to one vertex of the face
     * @param v2 a Tuple refering to one vertex of the face
     * @param v3 a Tuple refering to one vertex of the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(
        const Tuple& v1,
        const Tuple& v2,
        const Tuple& v3,
        int threadid = 0);
    /**
     * @brief a duplicate of ConcurrentTetMesh::try_set_face_mutex_two_ring usign the vids of the 3
     * vertices of a face
     *
     * @param v1 the vid of one vertex of the face
     * @param v2 the vid of one vertex of the face
     * @param v3 the vid of one vertex of the face
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_face_mutex_two_ring(size_t v1, size_t v2, size_t v3, int threadid = 0);
    /**
     * @brief try lock the one-ring neighboring traingles' incident vertices
     *
     * @param v Tuple refers to the vertex
     * @param threadid
     * @return true if all locked successfully
     */
    bool try_set_vertex_mutex_one_ring(const Tuple& v, int threadid = 0);

public:
    /**
     * @brief perform the given function for each edge
     *
     */
    void for_each_edge(const std::function<void(const TetMesh::Tuple&)>&) override;
    /**
     * @brief perform the given function for each vertex
     *
     */
    void for_each_vertex(const std::function<void(const TetMesh::Tuple&)>&) override;
    /**
     * @brief perform the given function for each tet
     *
     */
    void for_each_tetra(const std::function<void(const TetMesh::Tuple&)>&) override;
    int NUM_THREADS = 1;
};
} // namespace wmtk
