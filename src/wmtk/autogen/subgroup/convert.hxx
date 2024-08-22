#pragma once

#include "convert.hpp"
#include "subgroup_transformations.hpp"
namespace wmtk::autogen::subgroup {

    inline int8_t convert(PrimitiveType from, PrimitiveType to, int8_t source) {
        return convert(int8_t(from), int8_t(to), source);
    }
    inline int8_t convert(int8_t from, int8_t to, int8_t source) {
        return remap_table[from-1][to-1][source];
    }
}
