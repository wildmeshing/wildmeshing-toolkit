#pragma once

#include "detail/ets_id_counter.hpp"

namespace wmtk::threading {
// ---------------------------------------------------------------------------
// enumerable_thread_specific: replaces tbb::enumerable_thread_specific.
// Only `.local()` (and construction with an optional initial value) is used.
// Lock-free lookup: each thread owns a thread_local vector of slots.
// ---------------------------------------------------------------------------
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

} // namespace wmtk::threading