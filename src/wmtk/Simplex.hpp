#pragma once

#include "Primitive.hpp"
#include "Tuple.hpp"

namespace wmtk {

class Simplex
{
    PrimitiveType _ptype = PrimitiveType::Invalid;
    Tuple _tuple;

public:
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : _ptype{ptype}
        , _tuple{t}
    {}

    PrimitiveType primitive_type() const { return _ptype; }
    const Tuple& tuple() const { return _tuple; }
};
} // namespace wmtk