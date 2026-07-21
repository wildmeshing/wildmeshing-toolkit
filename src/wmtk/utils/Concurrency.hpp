#pragma once

// Drop-in replacements for the small subset of Intel TBB that wildmeshing-toolkit
// uses, implemented on top of the C++ standard library (std::thread & friends).
//
// The goal is a *mechanical* migration off TBB: every `tbb::X` used in the code
// base has a `wmtk::X` counterpart here with the same (used) API surface, so the
// only edits required elsewhere are `#include <tbb/...>` -> this header and
// `tbb::` -> `wmtk::`.
//
// Notes / intentional simplifications:
//  * Containers that used to grow concurrently (attribute / connectivity storage)
//    are NOT provided here -- they became plain std::vector, preallocated up front
//    (see TetMesh/TriMesh). What remains is collecting results out of a parallel
//    loop: `indexed_collector` where the loop already knows a unique slot per result
//    (no synchronisation at all, and a reproducible output order), and
//    `concurrent_vector` for the scattered appends that have no such slot.
//  * enumerable_thread_specific is lock-free on the hot path: each thread keeps a
//    small thread_local list of (instance-id -> value) slots. It cannot enumerate
//    them, though -- they live in thread_local storage, so the owning object cannot
//    reach other threads' copies.
//  * Thread count is an explicit argument to parallel_for. It used to travel through
//    a thread_local set by a task_arena shim, which meant a parallel_for's width
//    depended on invisible caller state -- and silently fell back to
//    hardware_concurrency() anywhere the arena had not been entered, including inside
//    threads spawned by task_group.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace wmtk {

// ---------------------------------------------------------------------------
// spin_mutex: replaces tbb::spin_mutex.
// Movable/copyable (resets to unlocked) so it can live inside elements of a
// std::vector that is grown single-threaded (e.g. the per-vertex mutex array).
// ---------------------------------------------------------------------------
class spin_mutex
{
    std::atomic<bool> m_locked{false};

public:
    spin_mutex() = default;
    spin_mutex(const spin_mutex&) noexcept {}
    spin_mutex(spin_mutex&&) noexcept {}
    spin_mutex& operator=(const spin_mutex&) noexcept { return *this; }
    spin_mutex& operator=(spin_mutex&&) noexcept { return *this; }

    void lock()
    {
        bool expected = false;
        while (!m_locked.compare_exchange_weak(
            expected,
            true,
            std::memory_order_acquire,
            std::memory_order_relaxed)) {
            expected = false;
            std::this_thread::yield();
        }
    }

    bool try_lock()
    {
        bool expected = false;
        return m_locked.compare_exchange_strong(expected, true, std::memory_order_acquire);
    }

    void unlock() { m_locked.store(false, std::memory_order_release); }
};

// ---------------------------------------------------------------------------
// enumerable_thread_specific: replaces tbb::enumerable_thread_specific.
// Only `.local()` (and construction with an optional initial value) is used.
// Lock-free lookup: each thread owns a thread_local vector of slots.
// ---------------------------------------------------------------------------
namespace detail {
inline std::atomic<std::uint64_t>& ets_id_counter()
{
    static std::atomic<std::uint64_t> counter{1};
    return counter;
}
} // namespace detail

template <typename T>
class enumerable_thread_specific
{
    struct Slot
    {
        std::uint64_t id;
        std::unique_ptr<T> value;
    };

    static std::vector<Slot>& thread_slots()
    {
        static thread_local std::vector<Slot> slots;
        return slots;
    }

    std::uint64_t m_id = detail::ets_id_counter().fetch_add(1, std::memory_order_relaxed);
    std::function<T()> m_factory;

public:
    enumerable_thread_specific()
        : m_factory([]() { return T(); })
    {}

    template <
        typename U,
        typename = std::enable_if_t<!std::is_same_v<std::decay_t<U>, enumerable_thread_specific>>>
    explicit enumerable_thread_specific(U&& init)
        : m_factory([captured = T(std::forward<U>(init))]() { return captured; })
    {}

    enumerable_thread_specific(const enumerable_thread_specific&) = delete;
    enumerable_thread_specific& operator=(const enumerable_thread_specific&) = delete;
    enumerable_thread_specific(enumerable_thread_specific&&) = delete;
    enumerable_thread_specific& operator=(enumerable_thread_specific&&) = delete;

    ~enumerable_thread_specific()
    {
        // Clear this (usually the main) thread's slot for this instance. Worker
        // threads are ephemeral in this shim, so their slots die with them.
        auto& slots = thread_slots();
        slots.erase(
            std::remove_if(
                slots.begin(),
                slots.end(),
                [this](const Slot& s) { return s.id == m_id; }),
            slots.end());
    }

    T& local()
    {
        auto& slots = thread_slots();
        for (auto& s : slots) {
            if (s.id == m_id) return *s.value;
        }
        slots.push_back(Slot{m_id, std::make_unique<T>(m_factory())});
        return *slots.back().value;
    }
};

// ---------------------------------------------------------------------------
// Collecting results out of a parallel loop. This is all tbb::concurrent_vector was
// ever used for here, and neither of the two shapes below needs a lock.
// ---------------------------------------------------------------------------

// indexed_collector: when the loop already knows a unique slot for each result --
// the edge collectors know it as `tup.eid(m)`, which is what the loop tests to decide
// which triangle owns an edge -- each thread writes to a slot no other thread can
// touch, so no synchronisation is required at all. compact() then returns the results
// in slot order, which does not depend on how the range was scheduled: unlike a
// mutex-guarded append, whose order is whichever thread got the lock first, this is
// reproducible run to run.
template <typename T>
class indexed_collector
{
    std::vector<T> m_slots;
    std::vector<char> m_filled;

public:
    explicit indexed_collector(std::size_t n)
        : m_slots(n)
        , m_filled(n, 0)
    {}

    // Callable concurrently as long as callers pass distinct `i`.
    void set(std::size_t i, const T& v)
    {
        m_slots[i] = v;
        m_filled[i] = 1;
    }

    std::vector<T> compact() const
    {
        std::vector<T> out;
        out.reserve(m_slots.size());
        for (std::size_t i = 0; i < m_slots.size(); ++i) {
            if (m_filled[i]) out.push_back(m_slots[i]);
        }
        return out;
    }
};

// concurrent_vector: the remaining shape, where results arrive from scattered points
// inside an operation rather than from an indexed sweep, so there is no slot to write
// to. Kept mutex-guarded over a std::vector. A per-thread-bucket version would be
// lock-free, but this shim's enumerable_thread_specific cannot enumerate its slots
// (they live in thread_local storage, so the owner cannot reach other threads'), and
// the remaining users append at most once per *failed mesh operation* -- the lock is
// nowhere near the cost of the operation that preceded it.
template <typename T>
class concurrent_vector
{
    std::vector<T> m_data;
    mutable std::mutex m_mutex;

public:
    using value_type = T;
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;
    using reference = typename std::vector<T>::reference;
    using const_reference = typename std::vector<T>::const_reference;

    concurrent_vector() = default;
    explicit concurrent_vector(std::size_t n)
        : m_data(n)
    {}
    concurrent_vector(std::size_t n, const T& value)
        : m_data(n, value)
    {}

    concurrent_vector(const concurrent_vector& o) { m_data = o.m_data; }
    concurrent_vector& operator=(const concurrent_vector& o)
    {
        m_data = o.m_data;
        return *this;
    }

    void push_back(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.push_back(v);
    }
    template <typename... Args>
    void emplace_back(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_data.emplace_back(std::forward<Args>(args)...);
    }

    // Single-threaded sizing/clearing (used outside parallel regions).
    void resize(std::size_t n) { m_data.resize(n); }
    void resize(std::size_t n, const T& value) { m_data.resize(n, value); }
    void clear() { m_data.clear(); }
    void reserve(std::size_t n) { m_data.reserve(n); }

    reference operator[](std::size_t i) { return m_data[i]; }
    const_reference operator[](std::size_t i) const { return m_data[i]; }

    std::size_t size() const { return m_data.size(); }
    bool empty() const { return m_data.empty(); }

    iterator begin() { return m_data.begin(); }
    iterator end() { return m_data.end(); }
    const_iterator begin() const { return m_data.begin(); }
    const_iterator end() const { return m_data.end(); }
};

// ---------------------------------------------------------------------------
// concurrent_priority_queue: replaces tbb::concurrent_priority_queue.
// Max-heap by default, matching TBB. std::priority_queue + mutex.
// Non-movable (holds a mutex); constructed in place inside std::vector(count).
// ---------------------------------------------------------------------------
template <typename T, typename Compare = std::less<T>>
class concurrent_priority_queue
{
    mutable std::mutex m_mutex;
    std::priority_queue<T, std::vector<T>, Compare> m_queue;

public:
    concurrent_priority_queue() = default;
    concurrent_priority_queue(const concurrent_priority_queue&) = delete;
    concurrent_priority_queue& operator=(const concurrent_priority_queue&) = delete;

    bool try_pop(T& out)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) return false;
        out = m_queue.top();
        m_queue.pop();
        return true;
    }

    void push(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(v);
    }

    template <typename... Args>
    void emplace(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.emplace(std::forward<Args>(args)...);
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }
};

// ---------------------------------------------------------------------------
// concurrent_queue: replaces tbb::concurrent_queue (FIFO). std::deque + mutex.
// ---------------------------------------------------------------------------
template <typename T>
class concurrent_queue
{
    mutable std::mutex m_mutex;
    std::deque<T> m_queue;

public:
    concurrent_queue() = default;
    concurrent_queue(const concurrent_queue&) = delete;
    concurrent_queue& operator=(const concurrent_queue&) = delete;

    bool try_pop(T& out)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_queue.empty()) return false;
        out = std::move(m_queue.front());
        m_queue.pop_front();
        return true;
    }
    void push(const T& v)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push_back(v);
    }
    template <typename... Args>
    void emplace(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.emplace_back(std::forward<Args>(args)...);
    }
    std::size_t unsafe_size() const { return m_queue.size(); }
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }
};

// ---------------------------------------------------------------------------
// A trivial "whole container" range object, returned by concurrent_map::range()
// and consumed by parallel_for below (iterated serially).
// ---------------------------------------------------------------------------
template <typename It>
class container_range
{
    It m_begin, m_end;

public:
    container_range(It b, It e)
        : m_begin(b)
        , m_end(e)
    {}
    It begin() const { return m_begin; }
    It end() const { return m_end; }
    bool empty() const { return m_begin == m_end; }
};

namespace detail {
template <typename MapType>
class concurrent_associative
{
protected:
    MapType m_map;
    mutable std::mutex m_mutex;

public:
    using key_type = typename MapType::key_type;
    using mapped_type = typename MapType::mapped_type;
    using value_type = typename MapType::value_type;
    using iterator = typename MapType::iterator;
    using const_iterator = typename MapType::const_iterator;

    concurrent_associative() = default;

    // std::map / std::unordered_map keep references to elements valid across
    // inserts, so returning a reference under a short structural lock is safe to
    // use afterwards without holding the lock (matches TBB semantics; concurrent
    // writes to the *same* element remain the caller's responsibility, as in TBB).
    mapped_type& operator[](const key_type& k)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map[k];
    }

    iterator find(const key_type& k)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.find(k);
    }
    const_iterator find(const key_type& k) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.find(k);
    }

    std::size_t count(const key_type& k) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_map.count(k);
    }

    iterator begin() { return m_map.begin(); }
    iterator end() { return m_map.end(); }
    const_iterator begin() const { return m_map.begin(); }
    const_iterator end() const { return m_map.end(); }

    std::size_t size() const { return m_map.size(); }
    bool empty() const { return m_map.empty(); }
    void clear() { m_map.clear(); }

    // Serial "range" over the whole map (see parallel_for(container_range,...)).
    container_range<iterator> range() { return {m_map.begin(), m_map.end()}; }
    container_range<iterator> range(std::size_t) { return {m_map.begin(), m_map.end()}; }
};
} // namespace detail

template <typename Key, typename T, typename Compare = std::less<Key>>
class concurrent_map : public detail::concurrent_associative<std::map<Key, T, Compare>>
{
};

template <typename Key, typename T, typename Hash = std::hash<Key>>
class concurrent_unordered_map
    : public detail::concurrent_associative<std::unordered_map<Key, T, Hash>>
{
};

// ---------------------------------------------------------------------------
// blocked_range: replaces tbb::blocked_range.
// ---------------------------------------------------------------------------
template <typename Index>
class blocked_range
{
    Index m_begin, m_end;

public:
    blocked_range(Index begin, Index end, std::size_t /*grainsize*/ = 1)
        : m_begin(begin)
        , m_end(end)
    {}
    Index begin() const { return m_begin; }
    Index end() const { return m_end; }
    bool empty() const { return !(m_begin < m_end); }
    std::size_t size() const { return static_cast<std::size_t>(m_end - m_begin); }
};

// ---------------------------------------------------------------------------
// parallel_for: replaces tbb::parallel_for.
// The thread count is an explicit argument: `num_threads > 0` runs on that many
// threads, anything else (the default 0) falls back to hardware_concurrency().
// ---------------------------------------------------------------------------
template <typename Index, typename Func>
void parallel_for(const blocked_range<Index>& range, Func&& func, int num_threads = 0)
{
    const Index begin = range.begin();
    const Index end = range.end();
    if (!(begin < end)) return;

    const std::size_t total = static_cast<std::size_t>(end - begin);

    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 1;
    int requested = num_threads;
    std::size_t nthreads = requested > 0 ? static_cast<std::size_t>(requested) : hw;
    nthreads = std::min<std::size_t>(nthreads, total);
    if (nthreads <= 1) {
        func(blocked_range<Index>(begin, end));
        return;
    }

    const std::size_t chunk = (total + nthreads - 1) / nthreads;

    std::exception_ptr eptr;
    std::mutex eptr_mutex;
    auto run = [&](Index cb, Index ce) {
        try {
            func(blocked_range<Index>(cb, ce));
        } catch (...) {
            std::lock_guard<std::mutex> lock(eptr_mutex);
            if (!eptr) eptr = std::current_exception();
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(nthreads - 1);
    std::size_t offset = 0;
    for (std::size_t t = 0; t < nthreads && offset < total; ++t) {
        const std::size_t cb = offset;
        const std::size_t ce = std::min(total, offset + chunk);
        offset = ce;
        const Index rb = begin + static_cast<Index>(cb);
        const Index re = begin + static_cast<Index>(ce);
        if (t + 1 == nthreads || offset >= total) {
            // run the last chunk on the calling thread
            run(rb, re);
        } else {
            workers.emplace_back([&run, rb, re]() { run(rb, re); });
        }
    }
    for (auto& w : workers) w.join();

    if (eptr) std::rethrow_exception(eptr);
}

// Serial overload used for iterating a concurrent_map via .range().
template <typename It, typename Func>
void parallel_for(container_range<It> range, Func&& func, int /*num_threads*/ = 0)
{
    func(range);
}

// ---------------------------------------------------------------------------
// task_group: replaces tbb::task_group. Each run() spawns a thread; wait() joins.
// ---------------------------------------------------------------------------
class task_group
{
    std::vector<std::thread> m_threads;
    std::exception_ptr m_eptr;
    std::mutex m_eptr_mutex;

public:
    task_group() = default;
    task_group(const task_group&) = delete;
    task_group& operator=(const task_group&) = delete;

    template <typename F>
    void run(F&& f)
    {
        m_threads.emplace_back([this, f = std::forward<F>(f)]() mutable {
            try {
                f();
            } catch (...) {
                std::lock_guard<std::mutex> lock(m_eptr_mutex);
                if (!m_eptr) m_eptr = std::current_exception();
            }
        });
    }

    void wait()
    {
        for (auto& t : m_threads) {
            if (t.joinable()) t.join();
        }
        m_threads.clear();
        if (m_eptr) {
            std::exception_ptr e = m_eptr;
            m_eptr = nullptr;
            std::rethrow_exception(e);
        }
    }

    ~task_group()
    {
        for (auto& t : m_threads) {
            if (t.joinable()) t.join();
        }
    }
};

} // namespace wmtk
