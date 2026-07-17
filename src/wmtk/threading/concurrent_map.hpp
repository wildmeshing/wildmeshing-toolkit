#pragma once

#include <map>

#include "detail/concurrent_associative.hpp"

namespace wmtk::threading {

template <typename Key, typename T, typename Compare = std::less<Key>>
class concurrent_map : public detail::concurrent_associative<std::map<Key, T, Compare>>
{
};

} // namespace wmtk::threading