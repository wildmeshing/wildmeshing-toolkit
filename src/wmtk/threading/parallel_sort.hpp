#pragma once

namespace wmtk::threading {

// ---------------------------------------------------------------------------
// parallel_sort: replaces tbb::parallel_sort (plain std::sort here).
// ---------------------------------------------------------------------------
template <typename RandomIt>
void parallel_sort(RandomIt first, RandomIt last)
{
    std::sort(first, last);
}

template <typename RandomIt, typename Compare>
void parallel_sort(RandomIt first, RandomIt last, Compare comp)
{
    std::sort(first, last, comp);
}

} // namespace wmtk::threading