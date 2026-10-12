#pragma once

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace wmtk::threading {

namespace detail {
inline std::atomic<std::uint64_t>& ets_id_counter()
{
    static std::atomic<std::uint64_t> counter{1};
    return counter;
}
} // namespace detail

// ---------------------------------------------------------------------------
// enumerable_thread_specific: replaces tbb::enumerable_thread_specific.
// Only `.local()` (and construction with an optional initial value) is used.
// Lock-free lookup: each thread owns a thread_local vector of slots.
//
// Slots outlive their object on other threads. The task_group workers persist for the life of
// the process (task_group.hpp), so a thread that used an instance keeps that instance's slot
// after the instance is destroyed; nothing else could remove it, since the slot vector belongs
// to that thread alone. Each slot therefore carries a weak reference to its instance, and a
// thread drops its dead slots -- destroying their values on the thread that created them --
// the next time it creates a slot. That keeps the lookup loop free of any liveness test, bounds
// what a dead instance leaves behind to one value per thread until that thread next meets a
// new instance (every new mesh is one), and ids are never reused, so a dead slot can never be
// mistaken for a live one in the meantime.
// ---------------------------------------------------------------------------
template <typename T>
class enumerable_thread_specific
{
    struct Slot
    {
        std::uint64_t id;
        std::weak_ptr<const void> owner;
        std::unique_ptr<T> value;
    };

    static std::vector<Slot>& thread_slots()
    {
        static thread_local std::vector<Slot> slots;
        return slots;
    }

    std::uint64_t m_id = detail::ets_id_counter().fetch_add(1, std::memory_order_relaxed);
    std::function<T()> m_factory;
    /// Expires when this instance is destroyed, which is how other threads learn of it.
    std::shared_ptr<const void> m_alive = std::make_shared<char>(0);

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
        // Clear this (usually the main) thread's slot for this instance. Other threads' slots
        // expire with m_alive and are dropped by those threads (see the class comment).
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
            if (s.id == m_id) {
                return *s.value;
            }
        }
        // No slot for this thread yet. Drop the slots of instances that no longer exist, then
        // create one.
        slots.erase(
            std::remove_if(
                slots.begin(),
                slots.end(),
                [](const Slot& s) { return s.owner.expired(); }),
            slots.end());
        slots.push_back(Slot{m_id, m_alive, std::make_unique<T>(m_factory())});
        return *slots.back().value;
    }
};

} // namespace wmtk::threading