#pragma once

#include <unordered_map>

#include "detail/concurrent_associative.hpp"

namespace wmtk::threading {

template <typename Key, typename T, typename Hash = std::hash<Key>>
class concurrent_unordered_map
    : public detail::concurrent_associative<std::unordered_map<Key, T, Hash>>
{
};

} // namespace wmtk::threading