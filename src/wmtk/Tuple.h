#pragma once

#include <tbb/enumerable_thread_specific.h>
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <tuple>
#include <wmtk/utils/Logger.hpp>

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
    friend Mesh::id(const Tuple& tuple, const PrimitiveType& type) const;
    friend Mesh::is_ccw(const Tuple& tuple) const;
    friend Mesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const;

    Tuple(long local_vid, long local_eid, long local_fid, long global_cid, long hash)
        : m_local_vid(local_vid)
        , m_local_eid(local_eid)
        , m_local_fid(local_fid)
        , m_global_cid(global_cid)
        , m_hash(hash)
    {}

    //         v2
    //       /    \
    //  e1  /      \  e0
    //     v0 - - - v1
    //         e2

    Tuple() = default;
    Tuple(const Tuple& other) = default;
    Tuple(Tuple&& other) = default;
    Tuple& operator=(const Tuple& other) = default;
    Tuple& operator=(Tuple&& other) = default;
};
} // namespace wmtk
