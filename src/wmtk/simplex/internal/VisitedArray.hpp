#pragma once

#include <vector>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/DynamicArray.hpp>

/**
 * This class is a utility to perform breadth first search on simplices.
 */

namespace wmtk::simplex::internal {

template <typename T, uint64_t ArraySize = 50>
class VisitedArray
{
public:
    bool is_visited(const T obj);

    const wmtk::utils::DynamicArray<T, ArraySize>& visited_array() const;

private:
    wmtk::utils::DynamicArray<T, ArraySize> m_visited;
};

template <typename T, uint64_t ArraySize>
inline bool VisitedArray<T, ArraySize>::is_visited(const T obj)
{
    for (const T& v : m_visited) {
        if (v == obj) {
            return true;
        }
    }
    m_visited.emplace_back(obj);
    return false;
}

template <typename T, uint64_t ArraySize>
inline auto VisitedArray<T, ArraySize>::visited_array() const
    -> const wmtk::utils::DynamicArray<T, ArraySize>&
{
    return m_visited;
}

} // namespace wmtk::simplex::internal
