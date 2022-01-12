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
        bool m_is_removed = false;

    public:
        void lock()
        {
            assert(!m_is_removed);
            mutex.lock();
        }

        bool trylock()
        {
            assert(!m_is_removed);
            return mutex.try_lock();
        }

        void unlock()
        {
            assert(!m_is_removed);
            mutex.unlock();
        }
    };

    ConcurrentTriMesh() = default;
    virtual ~ConcurrentTriMesh() = default;

    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
    {
        TriMesh::create_mesh(n_vertices, tris);
        m_vertex_mutex.grow_to_at_least(n_vertices);
    }

    // bool check_vertex_mutex_validation() { return m_vertex_mutex.size() == n_vertices(); }


private:
    tbb::concurrent_vector<VertexMutex> m_vertex_mutex;

    bool try_set_vertex_mutex(Tuple& v) { return m_vertex_mutex[v.vid()].trylock(); }
    bool try_set_vertex_mutex(size_t vid) { return m_vertex_mutex[vid].trylock(); }

    void set_vertex_mutex(Tuple& v) { m_vertex_mutex[v.vid()].lock(); }
    void set_vertex_mutex(size_t vid) { m_vertex_mutex[vid].lock(); }

    void unlock_vertex_mutex(Tuple& v) { m_vertex_mutex[v.vid()].unlock(); }
    void unlock_vertex_mutex(size_t vid) { m_vertex_mutex[vid].unlock(); }

protected:
    void resize_attributes(size_t v, size_t e, size_t t) override{
        m_vertex_mutex.grow_to_at_least(v);
    }


public:
    // bool collapse_edge(const Tuple& t, Tuple& new_t);

    size_t n_mutex() const { return m_vertex_mutex.size(); }

    size_t add_new_vertex_mutex()
    {
        // std::cout << "mutex_added" << std::endl;
        m_vertex_mutex.grow_by(1);
        return m_vertex_mutex.size() - 1;
    }

    int64_t release_vertex_mutex_in_stack(std::vector<size_t>& mutex_release_stack)
    {
        int num_released = 0;
        for (int i = mutex_release_stack.size() - 1; i >= 0; i--) {
            unlock_vertex_mutex(mutex_release_stack[i]);
            mutex_release_stack.pop_back();
            num_released++;
        }
        return num_released;
    }

    bool try_set_vertex_mutex_two_ring(Tuple& v, std::vector<size_t>& mutex_release_stack)
    {
        auto one_ring = get_one_ring_edges_for_vertex(v);
        // std::cout << "one_ring_size: " << one_ring.size() << std::endl;
        // std::cout << "one_ring_vertices: " << one_ring.size() << std::endl;
        for (auto v_one_ring : get_one_ring_edges_for_vertex(v)) {
            if (vector_contains(mutex_release_stack, v_one_ring.vid())) continue;
            if (try_set_vertex_mutex(v_one_ring)) {
                mutex_release_stack.push_back(v_one_ring.vid());
                for (auto v_two_ring : get_one_ring_edges_for_vertex(v_one_ring)) {
                    if (vector_contains(mutex_release_stack, v_two_ring.vid())) continue;
                    if (try_set_vertex_mutex(v_two_ring)) {
                        mutex_release_stack.push_back(v_two_ring.vid());
                    } else {
                        return false;
                    }
                }
            } else {
                return false;
            }
        }
        return true;
    }

    bool try_set_edge_mutex_two_ring(Tuple& e, std::vector<size_t>& mutex_release_stack)
    {
        // std::cout << "in_get_mutex" << std::endl;
        Tuple v1 = e;
        bool next_flag = false;

        // try v1
        if (try_set_vertex_mutex(v1)) {
            mutex_release_stack.push_back(v1.vid());
        } else {
            next_flag = true;
        }

        if (!v1.is_valid(*this)) {
            next_flag = true;
        }

        if (next_flag) {
            release_vertex_mutex_in_stack(mutex_release_stack);
            return false;
        }

        // std::cout << "get_v1_mutex" << std::endl;

        Tuple v2 = switch_vertex(e);

        // try v2
        if (!vector_contains(mutex_release_stack, v2.vid())) {
            if (try_set_vertex_mutex(v2)) {
                mutex_release_stack.push_back(v2.vid());
            } else {
                next_flag = true;
            }
        }

        if (!v2.is_valid(*this)) {
            next_flag = true;
        }

        if (next_flag) {
            release_vertex_mutex_in_stack(mutex_release_stack);
            return false;
        }

        // std::cout << "get_v2_mutex" << std::endl;


        // try v1 two ring
        next_flag = !try_set_vertex_mutex_two_ring(v1, mutex_release_stack);

        if (next_flag) {
            release_vertex_mutex_in_stack(mutex_release_stack);
            return false;
        }

        // std::cout << "get_v1_on_ring_mutex" << std::endl;


        // try v2 two ring
        next_flag = !try_set_vertex_mutex_two_ring(v2, mutex_release_stack);

        if (next_flag) {
            release_vertex_mutex_in_stack(mutex_release_stack);
            return false;
        }

        // std::cout << "get_all_mutex" << std::endl;
        return true;
    }
};
} // namespace wmtk
