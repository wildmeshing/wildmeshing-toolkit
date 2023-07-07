#pragma once

#include "Primitive.hpp"
#include "Tuple.hpp"

namespace wmtk {

class Simplex
{
    PrimitiveType _ptype;
    Tuple _tuple;

public:
    Simplex(const PrimitiveType& ptype, const Tuple& t)
        : _ptype{ptype}
        , _tuple{t}
    {}

    PrimitiveType primitive_type() const { return _ptype; }
    long dimension() const{
        return get_simplex_dimension(_ptype);
    }
    const Tuple& tuple() const { return _tuple; }
};
} // namespace wmtk
