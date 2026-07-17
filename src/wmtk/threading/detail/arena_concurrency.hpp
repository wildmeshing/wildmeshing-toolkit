#pragma once

namespace wmtk::threading::detail {

inline int& arena_concurrency()
{
    static thread_local int value = 0; // 0 => auto (hardware_concurrency)
    return value;
}

} // namespace wmtk::threading::detail