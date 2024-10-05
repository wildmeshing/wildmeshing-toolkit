#pragma once
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen::subgroup {

    // can return -1 if no mapping exists
    int8_t convert(PrimitiveType from, PrimitiveType to, int8_t source);
    int8_t convert(int8_t from, int8_t to, int8_t source);

    bool can_convert(PrimitiveType from, PrimitiveType to, int8_t source);


}

#include "convert.hxx"
