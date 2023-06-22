#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <tuple>
#include <wmtk/utils/Logger.hpp>
#include "Primitive.hpp"

namespace wmtk {
class Mesh;
class Tuple
{
private:
    // if Tuple is in 2d mesh m_global_cid is the global triangle id, and local_fid is -1
    // if Tuple is in 3d mesh m_global_cid is the global tetrahedron id
    long m_local_vid = -1;
    long m_local_eid = -1;
    long m_local_fid = -1;
    long m_global_cid = -1;
    long m_hash = -1;

public:
    friend class Mesh;
    //friend long Mesh::id(const Tuple& tuple, const PrimitiveType& type) const;
    //friend Mesh::is_ccw(const Tuple& tuple) const;
    //friend Mesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const;

    Tuple(long local_vid, long local_eid, long local_fid, long global_cid, long hash)
        : m_local_vid(local_vid)
        , m_local_eid(local_eid)
        , m_local_fid(local_fid)
        , m_global_cid(global_cid)
        , m_hash(hash)
    {}
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcomment"
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcomment"
#endif

    //         v2
    //       /    \
    //  e1  /      \  e0
    //     v0 - - - v1
    //         e2
#if defined(__clang__)
#pragma clang diagnostic pop
#elif (defined(__GNUC__) || defined(__GNUG__)) && !(defined(__clang__) || defined(__INTEL_COMPILER))
#pragma GCC diagnostic pop
#endif
    Tuple() = default;
    Tuple(const Tuple& other) = default;
    Tuple(Tuple&& other) = default;
    Tuple& operator=(const Tuple& other) = default;
    Tuple& operator=(Tuple&& other) = default;
};
} // namespace wmtk
