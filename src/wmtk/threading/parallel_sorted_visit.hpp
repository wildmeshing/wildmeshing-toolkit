#pragma once

#include <wmtk/threading/dynamic_parallel_for.hpp>

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace wmtk::threading {

/**
 * @brief Visit the elements of `v` in ascending order of `less`, in parallel over disjoint
 * ranges of keys.
 *
 * `v` is sorted in slices, one per task, in place. The key space is then cut into as many
 * parts, at splitters drawn from a regular sample of the sorted slices, and each part is merged
 * from its pieces of the slices and visited on a task of its own: `make_visitor()` is called
 * once per part, and the callable it returns is called with every element of that part, in
 * ascending order of `less`.
 *
 * `key_less` orders the keys the parts are cut at, and must be coarser than `less`: if
 * key_less(a, b) then less(a, b). All the elements with one key are then in one part, so a
 * visitor sees every element of each key it sees, in order -- which is what a "first element of
 * each key" scan needs. Which part a key falls in depends on the thread count; what a part's
 * visitor sees of each of its keys does not.
 *
 * Unlike a parallel merge sort this needs no second buffer the size of `v` -- the merge only
 * walks the slices. With one task (num_threads <= 1, or a short `v`) it is std::sort and one
 * visitor over the whole of `v`, on the calling thread.
 */
template <class T, class Less, class KeyLess, class MakeVisitor>
void parallel_sorted_visit(
    std::vector<T>& v,
    int num_threads,
    Less less,
    KeyLess key_less,
    MakeVisitor&& make_visitor)
{
    const size_t n = v.size();
    constexpr size_t kMinSlice = size_t(1) << 14;
    const size_t parts =
        std::min<size_t>(size_t(std::max(1, num_threads)), std::max<size_t>(1, n / kMinSlice));
    if (parts <= 1) {
        std::sort(v.begin(), v.end(), less);
        auto visit = make_visitor();
        for (const T& x : v) visit(x);
        return;
    }

    // 1. Sort the slices.
    std::vector<size_t> bound(parts + 1);
    for (size_t s = 0; s <= parts; ++s) bound[s] = n * s / parts;
    dynamic_parallel_for(parts, int(parts), 1, [&](size_t b, size_t e) {
        for (size_t s = b; s < e; ++s) {
            std::sort(v.begin() + bound[s], v.begin() + bound[s + 1], less);
        }
    });

    // 2. parts - 1 splitters, from `parts` evenly spaced elements of every slice.
    std::vector<T> sample;
    sample.reserve(parts * parts);
    for (size_t s = 0; s < parts; ++s) {
        const size_t len = bound[s + 1] - bound[s];
        for (size_t j = 0; j < parts; ++j) sample.push_back(v[bound[s] + len * j / parts]);
    }
    std::sort(sample.begin(), sample.end(), less);
    std::vector<T> split;
    split.reserve(parts - 1);
    for (size_t r = 1; r < parts; ++r) split.push_back(sample[r * sample.size() / parts]);
    std::vector<T>().swap(sample);

    // 3. Where each part starts in each slice: part r holds the keys in [split[r-1], split[r]),
    // so c[s * stride + r] is the first element of slice s not key_less than split[r-1]. The
    // splitters ascend, so the cuts do too.
    const size_t stride = parts + 1;
    std::vector<size_t> c(parts * stride);
    for (size_t s = 0; s < parts; ++s) {
        c[s * stride] = bound[s];
        c[s * stride + parts] = bound[s + 1];
        for (size_t r = 1; r < parts; ++r) {
            c[s * stride + r] = size_t(
                std::lower_bound(
                    v.begin() + c[s * stride + r - 1],
                    v.begin() + bound[s + 1],
                    split[r - 1],
                    key_less) -
                v.begin());
        }
    }

    // 4. Each part: a parts-way merge of its pieces.
    dynamic_parallel_for(parts, int(parts), 1, [&](size_t b, size_t e) {
        using Head = std::pair<const T*, const T*>; // next, end
        std::vector<Head> heads;
        heads.reserve(parts);
        const auto later = [&less](const Head& x, const Head& y) {
            return less(*y.first, *x.first);
        };
        for (size_t r = b; r < e; ++r) {
            auto visit = make_visitor();
            heads.clear();
            for (size_t s = 0; s < parts; ++s) {
                const size_t lo = c[s * stride + r], hi = c[s * stride + r + 1];
                if (lo < hi) heads.emplace_back(v.data() + lo, v.data() + hi);
            }
            std::make_heap(heads.begin(), heads.end(), later);
            while (!heads.empty()) {
                std::pop_heap(heads.begin(), heads.end(), later);
                Head& h = heads.back();
                visit(*h.first);
                if (++h.first == h.second) {
                    heads.pop_back();
                } else {
                    std::push_heap(heads.begin(), heads.end(), later);
                }
            }
        }
    });
}

} // namespace wmtk::threading
