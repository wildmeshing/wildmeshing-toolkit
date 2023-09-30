#pragma once
#include <wmtk/Tuple.hpp>

namespace wmtk::utils {
    struct TupleCellLessThan {
        bool operator()(const Tuple& a, const Tuple& b) const {
            return a.m_global_cid < b.m_global_cid;
        }
    };
}
