#include <wmtk/ConcurrentTriMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;

void ConcurrentTriMesh::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris)
{
    TriMesh::create_mesh(n_vertices, tris);
    m_vertex_mutex.grow_to_at_least(n_vertices);
}

int ConcurrentTriMesh::release_vertex_mutex_in_stack(std::vector<size_t>& mutex_release_stack)
{
    int num_released = 0;
    for (int i = mutex_release_stack.size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack[i]);
        mutex_release_stack.pop_back();
        num_released++;
    }
    return num_released;
}

bool ConcurrentTriMesh::try_set_vertex_mutex_two_ring(
    const Tuple& v,
    std::vector<size_t>& mutex_release_stack)
{
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

bool ConcurrentTriMesh::try_set_edge_mutex_two_ring(
    const Tuple& e,
    std::vector<size_t>& mutex_release_stack)
{
    Tuple v1 = e;
    bool release_flag = false;

    // try v1
    if (try_set_vertex_mutex(v1)) {
        mutex_release_stack.push_back(v1.vid());
    } else {
        release_flag = true;
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack(mutex_release_stack);
        return false;
    }

    // try v2
    Tuple v2 = switch_vertex(e);
    if (!vector_contains(mutex_release_stack, v2.vid())) {
        if (try_set_vertex_mutex(v2)) {
            mutex_release_stack.push_back(v2.vid());
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack(mutex_release_stack);
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v1, mutex_release_stack);

    if (release_flag) {
        release_vertex_mutex_in_stack(mutex_release_stack);
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v2, mutex_release_stack);

    if (release_flag) {
        release_vertex_mutex_in_stack(mutex_release_stack);
        return false;
    }

    return true;
}