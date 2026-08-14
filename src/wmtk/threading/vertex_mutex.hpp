#pragma once

#include <wmtk/threading/spin_mutex.hpp>

#include <atomic>
#include <limits>

namespace wmtk::threading {

/**
 * @brief A per-vertex lock plus the id of the thread currently holding it.
 *
 * One of these exists per vertex in TriMesh and TetMesh. Before running a topology-changing
 * operation a thread claims every vertex in the operation's one- or two-ring; on any failure it
 * releases what it has taken and the operation is retried later. `spin_mutex` provides the
 * mutual exclusion; the owner field exists only so that the ring walks can tell "already mine"
 * from "someone else's".
 *
 * That distinction is load-bearing because `spin_mutex` is not recursive. A two-ring walk visits
 * the same vertex many times (a vertex is in the one-ring of several of its neighbours), so
 * without the owner check the walk would try to re-lock vertices it already holds, fail, and
 * abort an operation that should have succeeded.
 *
 * @note This class was previously duplicated as a nested `VertexMutex` in both TriMesh.h and
 * TetMesh.h. The two copies drifted -- TriMesh's `unlock()` cleared the owner before releasing
 * the mutex and TetMesh's did it after, which is a bug (see below) -- so they are unified here.
 */
class VertexMutex
{
public:
    /// Sentinel stored in the owner field when no thread holds the vertex.
    static constexpr int no_owner() { return std::numeric_limits<int>::max(); }

    VertexMutex() = default;

    /**
     * Copy and move reset to unlocked-and-unowned rather than propagating state, mirroring
     * `spin_mutex`. These exist only so a `std::vector<VertexMutex>` can be grown by
     * `resize_vertex_mutex()`, which happens single-threaded during mesh (re)initialization
     * when nothing is held. `std::atomic` is neither copyable nor movable, so without these
     * the vector would not compile.
     */
    VertexMutex(const VertexMutex&) noexcept {}
    VertexMutex(VertexMutex&&) noexcept {}
    VertexMutex& operator=(const VertexMutex&) noexcept { return *this; }
    VertexMutex& operator=(VertexMutex&&) noexcept { return *this; }

    bool trylock() { return m_mutex.try_lock(); }

    void unlock()
    {
        // Clear the owner BEFORE releasing the mutex, never after.
        //
        // With the two swapped there is a window in which the vertex is free but still stamped
        // with the departing thread's id. Another thread can acquire it and stamp its own id
        // inside that window, and the departing thread's clear then lands on top -- wiping the
        // *new* owner's stamp. The new owner subsequently reads `no_owner()` for a vertex it
        // actually holds, tries to lock it again, fails (the spin_mutex is not recursive) and
        // aborts an operation that had already succeeded in claiming its whole ring.
        //
        // The window is only a couple of instructions wide, so it does not reproduce by timing
        // in an optimized build; under ThreadSanitizer it showed up as 24 violations in 28k
        // acquisitions. See the `vertex_mutex_owner_integrity` test.
        reset_owner();
        m_mutex.unlock();
    }

    /**
     * Read the owning thread id, or `no_owner()`.
     *
     * Deliberately callable without holding the lock -- that is precisely how the ring walks
     * use it -- which is why the field has to be atomic rather than a plain int.
     *
     * Relaxed ordering is sufficient, and the reasoning is worth keeping because the field
     * looks like it wants an acquire/release pair and does not need one:
     *
     *   * A thread only ever writes its *own* id into a vertex it holds, and clears it before
     *     releasing. A thread reading its own id back is reading its own most recent write to
     *     a single atomic object, which program order guarantees regardless of ordering.
     *   * A thread reading some *other* id, or a stale `no_owner()`, falls through to
     *     `trylock()`, and the spin_mutex's acquire/release pair is the real arbiter. A stale
     *     answer there costs one wasted attempt, never correctness.
     *
     * The mutual exclusion and the memory visibility of everything the operation touches come
     * from `spin_mutex`, not from this field.
     */
    int get_owner() const { return m_owner.load(std::memory_order_relaxed); }

    void set_owner(int n) { m_owner.store(n, std::memory_order_relaxed); }

    void reset_owner() { m_owner.store(no_owner(), std::memory_order_relaxed); }

private:
    spin_mutex m_mutex;
    std::atomic<int> m_owner{no_owner()};
};

} // namespace wmtk::threading
