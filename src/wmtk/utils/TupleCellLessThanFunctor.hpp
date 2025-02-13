#pragma once
#include <wmtk/Tuple.hpp>

namespace wmtk::utils {
class TupleCellLessThan
{
public:
    bool operator()(const Tuple& a, const Tuple& b) const
    {
        return a.global_cid() < b.global_cid();
    }
};
} // namespace wmtk::utils
