#include <wmtk/ConcurrentTetMesh.h>

#include <Tracy.hpp>

void wmtk::ConcurrentTetMesh::init(
    size_t n_vertices,
    const std::vector<std::array<size_t, 4>>& tets)
{
    wmtk::TetMesh::init(n_vertices, tets);
    m_vertex_mutex.grow_to_at_least(n_vertices);
}


int wmtk::ConcurrentTetMesh::release_vertex_mutex_in_stack()
{
    ZoneScoped;
    int num_released = 0;
    for (int i = mutex_release_stack.local().size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack.local()[i]);
        // mutex_release_stack.local().pop_back();
        num_released++;
    }
    mutex_release_stack.local().clear();
    return num_released;
}

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring(
    const Tuple& v,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    ZoneScoped;
    for (auto v_one_ring : get_one_ring_vertices_for_vertex(v)) {
        // if (vector_contains(mutex_release_stack.local(), v_one_ring.vid(*this))) continue;
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring.vid(*this));
            for (auto v_two_ring : get_one_ring_vertices_for_vertex(v_one_ring)) {
                // if (vector_contains(mutex_release_stack.local(), v_two_ring.vid(*this)))
                // continue;
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    mutex_release_stack.local().push_back(v_two_ring.vid(*this));
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

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid(
    const Tuple& v,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    ZoneScoped;
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v.vid(*this), get_one_ring_cache.local())) {
        // if (vector_contains(mutex_release_stack.local(), v_one_ring)) continue;
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            {
                ZoneScoped;
                mutex_release_stack.local().push_back(v_one_ring);
            }
            for (auto v_two_ring : get_one_ring_vids_for_vertex(v_one_ring, get_one_ring_cache.local())) {
                // if (vector_contains(mutex_release_stack.local(), v_two_ring)) continue;
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    ZoneScoped;
                    mutex_release_stack.local().push_back(v_two_ring);
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

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid(
    size_t v,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v, get_one_ring_cache.local())) {
        // if (vector_contains(mutex_release_stack.local(), v_one_ring)) continue;
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring);
            for (auto v_two_ring : get_one_ring_vids_for_vertex(v_one_ring, get_one_ring_cache.local())) {
                // if (vector_contains(mutex_release_stack.local(), v_two_ring)) continue;
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    mutex_release_stack.local().push_back(v_two_ring);
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


bool wmtk::ConcurrentTetMesh::try_set_edge_mutex_two_ring(
    const Tuple& e,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    ZoneScoped;
    Tuple v1 = e;
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    Tuple v2 = switch_vertex(e);
    // if (!vector_contains(mutex_release_stack.local(), v2.vid(*this))) {
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_face_mutex_two_ring(
    const Tuple& f,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    ZoneScoped;
    Tuple v1 = f;
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    Tuple v2 = switch_vertex(f);
    // if (!vector_contains(mutex_release_stack, v2.vid(*this))) {
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3
    Tuple v3 = switch_edge(v2).switch_vertex(*this);
    // if (!vector_contains(mutex_release_stack, v3.vid(*this))) {
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            mutex_release_stack.local().push_back(v3.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v3.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v3, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_face_mutex_two_ring(
    const Tuple& v1,
    const Tuple& v2,
    const Tuple& v3,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    // if (!vector_contains(mutex_release_stack.local(), v2.vid(*this))) {
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3
    // if (!vector_contains(mutex_release_stack.local(), v3.vid(*this))) {
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            mutex_release_stack.local().push_back(v3.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v3.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v3, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_face_mutex_two_ring(
    size_t v1,
    size_t v2,
    size_t v3,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1);
        } else {
            release_flag = true;
        }
    }

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    // if (!vector_contains(mutex_release_stack.local(), v2)) {
    if (m_vertex_mutex[v2].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2);
        } else {
            release_flag = true;
        }
    }

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3
    // if (!vector_contains(mutex_release_stack, v3)) {
    if (m_vertex_mutex[v3].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            mutex_release_stack.local().push_back(v3);
        } else {
            release_flag = true;
        }
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v3, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_one_ring(
    const Tuple& v,
    // std::vector<size_t>& mutex_release_stack,
    int threadid)
{
    // if (!vector_contains(mutex_release_stack, v.vid(*this))) {
    if (m_vertex_mutex[v.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v, threadid)) {
            mutex_release_stack.local().push_back(v.vid(*this));
            for (auto v_one_ring : get_one_ring_vids_for_vertex(v.vid(*this), get_one_ring_cache.local())) {
                // if (!vector_contains(mutex_release_stack.local(), v_one_ring)) {
                if (m_vertex_mutex[v_one_ring].get_owner() != threadid) {
                    if (try_set_vertex_mutex(v_one_ring, threadid)) {
                        mutex_release_stack.local().push_back(v_one_ring);
                    } else {
                        release_vertex_mutex_in_stack();
                        return false;
                    }
                }
            }
        } else {
            release_vertex_mutex_in_stack();
            return false;
        }
    }
    return true;
}
