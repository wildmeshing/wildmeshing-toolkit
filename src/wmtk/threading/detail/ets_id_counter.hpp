#pragma once

#include <atomic>
#include <thread>

namespace wmtk::threading::detail {

inline std::atomic<std::uint64_t>& ets_id_counter()
{
    static std::atomic<std::uint64_t> counter{1};
    return counter;
}

} // namespace wmtk::threading::detail