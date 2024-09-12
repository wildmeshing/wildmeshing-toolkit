#pragma once
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::autogen::subgroup {

    int8_t convert(PrimitiveType from, PrimitiveType to, int8_t source);
    int8_t convert(int8_t from, int8_t to, int8_t source);

}

#include "convert.hxx"
