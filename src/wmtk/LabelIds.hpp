#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace wmtk {

/// The id of an input (a label): tetwild's and triwild's input files, simwild's face tags and tet
/// tags, the offset's labels. At most 65535 per mesh.
using LabelId = uint16_t;
/// How many labels a mesh can have: one id each.
constexpr size_t max_labels = size_t(1) << (8 * sizeof(LabelId));

namespace detail {

/**
 * @brief A sorted set of elements with distinct keys, held inline up to InlineN and on the heap
 * beyond: the storage behind LabelIds and LabelCounts.
 *
 * Why not a bitset: a simplex lies on one input almost always and on two or three where inputs
 * meet, whatever the number of inputs, so the record costs the same for one label as for
 * thousands. Until 2026-11 the inputs were a 64-bit mask (InputSet), one bit per input, and a
 * mesh with more than 64 labels was refused (scan_1, 78 cells).
 *
 * Bytes, not a union with a pointer: the records stay aligned as Elem (2), not as a pointer
 * (8). The heap block, when there is one, is new Elem[capacity] (at most 255 elements on one
 * simplex) and its pointer is memcpy'd into the bytes. Copies are deep, so the attribute
 * collections' copies and rollbacks need nothing else. Elem is trivially copyable; Key(e) gives its
 * key.
 */
template <typename Elem, size_t InlineN, typename KeyOf>
class SmallSortedSet
{
    static constexpr size_t inline_bytes = InlineN * sizeof(Elem);
    static constexpr size_t storage_bytes = inline_bytes > sizeof(Elem*) ? inline_bytes
                                                                         : sizeof(Elem*);

public:
    using key_type = decltype(KeyOf{}(Elem{}));

    SmallSortedSet() = default;
    SmallSortedSet(const SmallSortedSet& o) { assign(o.begin(), o.end()); }
    SmallSortedSet(SmallSortedSet&& o) noexcept
    {
        std::memcpy(m_bytes, o.m_bytes, storage_bytes);
        m_count = o.m_count;
        m_capacity = o.m_capacity;
        o.m_count = 0;
        o.m_capacity = 0;
    }
    SmallSortedSet& operator=(const SmallSortedSet& o)
    {
        if (this != &o) assign(o.begin(), o.end());
        return *this;
    }
    SmallSortedSet& operator=(SmallSortedSet&& o) noexcept
    {
        if (this != &o) {
            free_heap();
            std::memcpy(m_bytes, o.m_bytes, storage_bytes);
            m_count = o.m_count;
            m_capacity = o.m_capacity;
            o.m_count = 0;
            o.m_capacity = 0;
        }
        return *this;
    }
    ~SmallSortedSet() { free_heap(); }

    size_t count() const { return m_count; }
    bool none() const { return m_count == 0; }
    void clear()
    {
        free_heap();
        m_count = 0;
    }

    const Elem* begin() const { return data(); }
    const Elem* end() const { return data() + m_count; }

    /// The element of key `k`, or nullptr.
    const Elem* find(key_type k) const
    {
        const Elem* it = lower(k);
        return it != end() && KeyOf{}(*it) == k ? it : nullptr;
    }
    bool contains(key_type k) const { return find(k) != nullptr; }

    /// Insert `e`; an element of the same key is replaced.
    void insert_or_assign(const Elem& e)
    {
        const key_type k = KeyOf{}(e);
        const size_t i = size_t(lower(k) - begin());
        if (i < m_count && KeyOf{}(data()[i]) == k) {
            data()[i] = e;
            return;
        }
        reserve(size_t(m_count) + 1);
        Elem* d = data();
        std::memmove(d + i + 1, d + i, (m_count - i) * sizeof(Elem));
        d[i] = e;
        ++m_count;
    }
    /// Remove the element of key `k`, if any.
    void erase(key_type k)
    {
        Elem* d = data();
        const size_t i = size_t(lower(k) - begin());
        if (i == m_count || KeyOf{}(d[i]) != k) return;
        std::memmove(d + i, d + i + 1, (m_count - i - 1) * sizeof(Elem));
        --m_count;
    }
    template <typename Pred>
    void erase_if(Pred pred)
    {
        Elem* d = data();
        size_t n = 0;
        for (size_t i = 0; i < m_count; ++i)
            if (!pred(d[i])) d[n++] = d[i];
        m_count = uint8_t(n);
    }

    bool operator==(const SmallSortedSet& o) const
    {
        return m_count == o.m_count && std::memcmp(begin(), o.begin(), m_count * sizeof(Elem)) == 0;
    }
    bool operator!=(const SmallSortedSet& o) const { return !(*this == o); }

protected:
    Elem* data() { return on_heap() ? heap() : reinterpret_cast<Elem*>(m_bytes); }
    const Elem* data() const { return on_heap() ? heap() : reinterpret_cast<const Elem*>(m_bytes); }
    const Elem* lower(key_type k) const
    {
        return std::lower_bound(begin(), end(), k, [](const Elem& e, key_type key) {
            return KeyOf{}(e) < key;
        });
    }
    void assign(const Elem* first, const Elem* last)
    {
        const size_t n = size_t(last - first);
        if (first == begin()) return;
        clear();
        reserve(n);
        std::memmove(data(), first, n * sizeof(Elem));
        m_count = uint8_t(n);
    }

private:
    bool on_heap() const { return m_capacity > 0; }
    Elem* heap() const
    {
        Elem* p;
        std::memcpy(&p, m_bytes, sizeof(p));
        return p;
    }
    void reserve(size_t n)
    {
        if (n > 0xFF) throw std::length_error("a simplex on more than 255 labels");
        if (n <= (on_heap() ? size_t(m_capacity) : InlineN)) return;
        const size_t cap = std::max<size_t>(n, 2 * std::max<size_t>(InlineN, m_capacity));
        const size_t c = std::min<size_t>(cap, 0xFF);
        Elem* p = new Elem[c];
        std::memcpy(p, data(), m_count * sizeof(Elem));
        free_heap();
        std::memcpy(m_bytes, &p, sizeof(p));
        m_capacity = uint8_t(c);
    }
    void free_heap()
    {
        if (!on_heap()) return;
        delete[] heap();
        m_capacity = 0;
    }

    alignas(Elem) unsigned char m_bytes[storage_bytes] = {};
    uint8_t m_count = 0;
    uint8_t m_capacity = 0; // 0 = inline
};

struct LabelIdKey
{
    LabelId operator()(LabelId l) const { return l; }
};

/// One input's orientation count on a simplex (WMTK_PER_INPUT_ORIENTATION).
struct LabelCount
{
    LabelId label = 0;
    int8_t count = 0;
};
struct LabelCountKey
{
    LabelId operator()(const LabelCount& c) const { return c.label; }
};

} // namespace detail

/**
 * @brief The labels (inputs) a simplex lies on: sorted, distinct ids. What the surface records
 * carry (SurfaceTagAttributes::labels()), what the envelopes are asked about
 * (InputEnvelopes::containment()), and what the feature edges and points carry. The
 * invariant: a tracked simplex stays within eps of EVERY label it has.
 */
class LabelIds : public detail::SmallSortedSet<LabelId, 4, detail::LabelIdKey>
{
public:
    LabelIds() = default;
    LabelIds(std::initializer_list<LabelId> ids)
    {
        for (const LabelId l : ids) insert(l);
    }
    void insert(LabelId l) { insert_or_assign(l); }
    /// The union with `o`.
    void merge(const LabelIds& o)
    {
        for (const LabelId l : o) insert(l);
    }
    /// The lowest id. Not on an empty set.
    LabelId lowest() const { return *begin(); }
    /// Whether the two share a label.
    bool intersects(const LabelIds& o) const
    {
        const LabelId *a = begin(), *b = o.begin();
        while (a != end() && b != o.end()) {
            if (*a == *b) return true;
            *a < *b ? ++a : ++b;
        }
        return false;
    }
    /// Whether every label of this one is one of `o`'s.
    bool subset_of(const LabelIds& o) const
    {
        return std::all_of(begin(), end(), [&](LabelId l) { return o.contains(l); });
    }
    /// Lexicographic on the sorted ids: a map key.
    bool operator<(const LabelIds& o) const
    {
        return std::lexicographical_compare(begin(), end(), o.begin(), o.end());
    }
};

/// Per label, its orientation count (WMTK_PER_INPUT_ORIENTATION): sorted by label, no zero
/// counts stored.
using LabelCounts = detail::SmallSortedSet<detail::LabelCount, 2, detail::LabelCountKey>;

} // namespace wmtk
