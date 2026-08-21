#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

namespace wmtk {

/// Returned by SlotPool::request, and by the meshes' request_*_slots / get_next_empty_slot_*
/// entry points, when no slot could be reserved.
inline constexpr size_t INVALID_SLOT = static_cast<size_t>(-1);

/**
 * @brief Preallocated storage plus the atomic counter that hands slots out of it.
 *
 * The mesh classes keep their connectivity in a vector that is deliberately larger than the
 * live mesh: `[0, live())` is filled, and `[live(), capacity())` is spare headroom that
 * operations consume. Operations run in parallel, so the counter is advanced with a CAS loop
 * and a request that would run past `capacity()` is REFUSED rather than served by growing the
 * storage -- growing means reallocating, which would move the connectivity out from under
 * concurrent readers. Callers ask for their slots up front and abort before mutating when the
 * answer is a refusal.
 *
 * Pairing the counter with the storage is the point of this class. The invariant that matters
 * is `live() <= capacity()`, and it can only be checked -- or maintained -- where both are in
 * reach. It is also the seat for any future growth strategy: implementing one here covers
 * every allocation site in both meshes at once.
 *
 * Not copyable or movable, because of the atomic. The meshes that hold these were already
 * neither, for the same reason.
 *
 * @note Only `request` is safe to call concurrently. Everything else --
 * `resize`, `clear`, `set_live`, `shrink_to_fit` -- is for the serial points between passes,
 * and `operator[]` is subject to the usual rule that a reader must not race the grower.
 */
template <class T>
class SlotPool
{
public:
    using value_type = T;

    SlotPool() = default;
    SlotPool(const SlotPool&) = delete;
    SlotPool& operator=(const SlotPool&) = delete;

    /// Slots the storage can hold: the end of the spare region.
    size_t capacity() const { return m_data.size(); }

    /// Slots handed out so far: the end of the live region, and the next index `request`
    /// would return. Named apart from `capacity()` because the two are genuinely different
    /// numbers, and conflating them is what makes the spare region hard to reason about.
    size_t live() const { return m_live.load(std::memory_order_relaxed); }

    /// Set the live count directly. For init and consolidate, which fill `[0, n)` themselves.
    void set_live(size_t n) { m_live.store(n, std::memory_order_relaxed); }

    /// Resize the storage. Does not touch the live count -- the caller decides that.
    void resize(size_t n) { m_data.resize(n); }

    void shrink_to_fit() { m_data.shrink_to_fit(); }

    /// Drop the storage and the live count together, leaving an empty pool.
    void clear()
    {
        m_data.clear();
        set_live(0);
    }

    T& operator[](size_t i) { return m_data[i]; }
    const T& operator[](size_t i) const { return m_data[i]; }

    /**
     * @name Whole-storage iteration
     *
     * These span `[0, capacity())`, spare region included -- they are the vector's iterators,
     * not the live range. Callers that want only live slots index `[0, live())` instead.
     * @{
     */
    auto begin() { return m_data.begin(); }
    auto end() { return m_data.end(); }
    auto begin() const { return m_data.begin(); }
    auto end() const { return m_data.end(); }
    /** @} */

    /**
     * @brief Atomically reserve `n` contiguous fresh slots.
     *
     * @return The first index of the block, or @ref INVALID_SLOT if serving it would run past
     * `capacity()`. A refusal leaves the counter untouched, so the slots that remain stay
     * available to a later, smaller request; a request that leaked would push `live()` past
     * `capacity()` and send subsequent iteration out of bounds.
     *
     * `n == 0` reports the current `live()` without consuming anything.
     */
    size_t request(size_t n)
    {
        size_t first = m_live.load(std::memory_order_relaxed);
        if (n == 0) {
            return first;
        }
        const size_t cap = m_data.size();
        do {
            // Phrased as a subtraction rather than `first + n > cap` so that it cannot
            // overflow; `first <= cap` always holds, so `cap - first` is well defined.
            if (first > cap || n > cap - first) {
                return INVALID_SLOT;
            }
        } while (!m_live.compare_exchange_weak(
            first,
            first + n,
            std::memory_order_acq_rel,
            std::memory_order_relaxed));

        // Reset the handed-out slots. The storage keeps whatever a previous generation left in
        // the spare region, so a fresh slot has to be cleared here rather than being fresh by
        // construction -- which is what the tbb::concurrent_vector this replaced used to give.
        for (size_t i = first; i < first + n; ++i) {
            m_data[i] = T{};
        }
        return first;
    }

private:
    std::vector<T> m_data;
    std::atomic<size_t> m_live{0};
};

} // namespace wmtk
