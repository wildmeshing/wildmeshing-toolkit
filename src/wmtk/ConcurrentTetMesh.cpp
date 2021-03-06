#include <wmtk/ConcurrentTetMesh.h>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/parallel_for.h>
#include <Tracy.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

void wmtk::ConcurrentTetMesh::init(
    size_t n_vertices,
    const std::vector<std::array<size_t, 4>>& tets)
{
    wmtk::TetMesh::init(n_vertices, tets);
    m_vertex_mutex.grow_to_at_least(n_vertices);
}


int wmtk::ConcurrentTetMesh::release_vertex_mutex_in_stack()
{
    int num_released = 0;
    auto& stack = mutex_release_stack.local();
    for (int i = stack.size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(stack[i]);
        num_released++;
    }
    stack.clear();
    return num_released;
}

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vertices_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            stack.push_back(v_one_ring.vid(*this));
            for (auto v_two_ring : get_one_ring_vertices_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring.vid(*this));
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

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid)
{
    auto& cache = get_one_ring_cache.local();
    auto& stack = mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v.vid(*this), cache)) {
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            {
                stack.push_back(v_one_ring);
            }
            for (auto v_two_ring :
                 get_one_ring_vids_for_vertex(v_one_ring, cache)) {
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring);
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

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring_vid(size_t v, int threadid)
{
    auto& cache = get_one_ring_cache.local();
    auto& stack =  mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v, cache)) {
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            stack.push_back(v_one_ring);
            for (auto v_two_ring :
                 get_one_ring_vids_for_vertex(v_one_ring, cache)) {
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring);
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


bool wmtk::ConcurrentTetMesh::try_set_edge_mutex_two_ring(const Tuple& e, int threadid)
{
    const Tuple& v1 = e;
    auto& stack = mutex_release_stack.local();

    stack.reserve(128);

    // try v1
    auto acquire_lock = [&]() {
        if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
            if (try_set_vertex_mutex(v1, threadid)) {
                stack.push_back(v1.vid(*this));
            } else {
                return false;
            }
        }
        if (!v1.is_valid(*this)) {
            return false;
        }

        // try v2
        Tuple v2 = switch_vertex(v1);
        if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
            if (try_set_vertex_mutex(v2, threadid)) {
                stack.push_back(v2.vid(*this));
            } else {
                return false;
            }
        }
        if (!v2.is_valid(*this)) {
            return false;
        }

        // try v1 two ring
        return (
            try_set_vertex_mutex_two_ring_vid(v1, threadid) &&
            try_set_vertex_mutex_two_ring_vid(v2, threadid));
    };

    if (!acquire_lock()) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_face_mutex_two_ring(const Tuple& f, int threadid)
{
    Tuple v1 = f;
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();


    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            stack.push_back(v1.vid(*this));
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
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            stack.push_back(v2.vid(*this));
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
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            stack.push_back(v3.vid(*this));
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
    int threadid)
{
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            stack.push_back(v1.vid(*this));
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
    // if (!vector_contains(stack, v2.vid(*this))) {
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            stack.push_back(v2.vid(*this));
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
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            stack.push_back(v3.vid(*this));
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
    int threadid)
{
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();

    auto try_all = [&]() {
        for (auto vv : {v1, v2, v3}) {
            if (m_vertex_mutex[vv].get_owner() != threadid) {
                if (try_set_vertex_mutex(vv, threadid)) {
                    stack.push_back(vv);
                } else {
                    return false;
                }
            }
        }

        for (auto vv : {v1, v2, v3}) {
            if (try_set_vertex_mutex_two_ring_vid(vv, threadid) == false) {
                return false;
            };
        }
        return true;
    };


    if (try_all() == false) {
        release_vertex_mutex_in_stack();
        return false;
    }

    return true;
}

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_one_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    auto& cache = get_one_ring_cache.local();
    if (m_vertex_mutex[v.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v, threadid)) {
            stack.push_back(v.vid(*this));
            for (auto v_one_ring :
                 get_one_ring_vids_for_vertex(v.vid(*this), cache)) {
                if (m_vertex_mutex[v_one_ring].get_owner() != threadid) {
                    if (try_set_vertex_mutex(v_one_ring, threadid)) {
                        stack.push_back(v_one_ring);
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

void wmtk::ConcurrentTetMesh::for_each_edge(const std::function<void(const TetMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tet_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    if (!tuple_from_tet(i).is_valid(*this)) continue;
                    for (int j = 0; j < 6; j++) {
                        auto tup = tuple_from_edge(i, j);
                        if (tup.eid(*this) == 6 * i + j) {
                            func(tup);
                        }
                    }
                }
            });
    });
}


void wmtk::ConcurrentTetMesh::for_each_tetra(const std::function<void(const TetMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tet_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_tet(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            });
    });
}


void wmtk::ConcurrentTetMesh::for_each_vertex(
    const std::function<void(const TetMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, vert_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_vertex(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            });
    });
}