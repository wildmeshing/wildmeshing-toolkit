#pragma once

#include <cassert>
#include "convert.hpp"
#include "subgroup_transformations.hpp"
namespace wmtk::autogen::subgroup {

inline int8_t convert(PrimitiveType from, PrimitiveType to, int8_t source)
{
    return convert(int8_t(from), int8_t(to), source);
}
inline int8_t convert(int8_t from, int8_t to, int8_t source)
{
    const int8_t v = remap_table[from - 1][to - 1][source];
    //assert(v != -1);
    return v;
}
    bool can_convert(PrimitiveType from, PrimitiveType to, int8_t source) {
        return convert(from,to,source) != -1;
    }
} // namespace wmtk::autogen::subgroup
