#pragma once
#include "AdaptiveTessellation.h"

namespace adaptive_tessellation {

template <typename OpType>
std::vector<wmtk::TriMesh::Tuple>
modified_tuples_T(const AdaptiveTessellation& m, const OpType& primary, const OpType& secondary)
{
    std::vector<wmtk::TriMesh::Tuple> ret = primary.modified_tuples(m);
    if (bool(secondary)) {
        const std::vector<wmtk::TriMesh::Tuple> mret = secondary.modified_tuples(m);
        ret.insert(ret.end(), mret.begin(), mret.end());
    }
    return ret;
}

template <typename OpType>
bool operation_success_T(const OpType& primary, const OpType& secondary, bool has_mirror)
{
    const bool normal_ok = bool(primary);
    const bool mirror_ok = !has_mirror && bool(secondary);
    return normal_ok && mirror_ok;
}
} // namespace adaptive_tessellation
