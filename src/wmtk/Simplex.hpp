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

    int global_id() const
    {
        throw "use the one in Mesh";
        return -1;
    }
    PrimitiveType primitive_type() const { return _ptype; }
    const Tuple& tuple() const { return _tuple; }

    bool operator<(const Simplex& rhs) const
    {
        throw "use compare function in Mesh";
        if (_ptype < rhs._ptype) {
            return true;
        }
        if (_ptype > rhs._ptype) {
            return false;
        }
        return global_id() < rhs.global_id();
    }

    bool operator==(const Simplex& rhs) const
    {
        throw "use compare function in Mesh";
        return (_ptype == rhs._ptype) && (global_id() == rhs.global_id());
    }
};
} // namespace wmtk