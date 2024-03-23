#pragma once

#include "PrimitiveType.hpp"


namespace wmtk {

class Mesh;
class PointMesh;
class TriMesh;
class EdgeMesh;
class TetMesh;

namespace components::internal {
class MultiMeshFromTag;
}

namespace attribute {
template <typename T, typename MeshType>
class Accessor;
}
namespace utils {
class TupleInspector;
}
namespace operations {
class Operation;
}
namespace utils {
// for identifying unique top level simplices between tuples
class TupleCellLessThan;
} // namespace utils
namespace multimesh {
class MultiMeshManager;
}

class Tuple
{
private:
    // if Tuple is in 2d mesh m_global_cid is the global triangle id, and local_fid is -1
    // if Tuple is in 3d mesh m_global_cid is the global tetrahedron id
    int64_t m_global_cid = -1;
    int8_t m_local_vid = -1;
    int8_t m_local_eid = -1;
    int8_t m_local_fid = -1;
    int8_t m_hash = -1;

public:
    friend class Mesh;
    friend class PointMesh;
    friend class EdgeMesh;
    friend class TriMesh;
    friend class TetMesh;
    friend class multimesh::MultiMeshManager;
    template <typename T, typename MeshType>
    friend class attribute::Accessor;
    friend class operations::Operation;
    friend class utils::TupleCellLessThan;
    friend class utils::TupleInspector;
    friend class components::internal::MultiMeshFromTag;
    // friend int64_t Mesh::id(const Tuple& tuple, const PrimitiveType& type) const;
    // friend Mesh::is_ccw(const Tuple& tuple) const;
    // friend Mesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const;

    Tuple(int8_t local_vid, int8_t local_eid, int8_t local_fid, int64_t global_cid, int8_t hash);

    //         v2
    //       /    \.
    //  e1  /      \  e0
    //     v0 - - - v1
    //         e2

    Tuple() = default;
    Tuple(const Tuple& other) = default;
    Tuple(Tuple&& other) = default;
    Tuple& operator=(const Tuple& other) = default;
    Tuple& operator=(Tuple&& other) = default;

    bool operator==(const Tuple& t) const;
    bool operator!=(const Tuple& t) const;
    bool operator<(const Tuple& t) const;
    /// Checks whether two tuples are equal, but ignores the hash
    bool same_ids(const Tuple& t) const;

    /// Checks if a tuple is "null". This merely implies the global index is -1
    bool is_null() const;
    Tuple with_updated_hash(int64_t new_hash) const;

private:
    int8_t local_vid() const;
    int8_t local_eid() const;
    int8_t local_fid() const;
};
inline Tuple::Tuple(
    int8_t local_vid,
    int8_t local_eid,
    int8_t local_fid,
    int64_t global_cid,
    int8_t hash)
    : m_global_cid(global_cid)
    , m_local_vid(local_vid)
    , m_local_eid(local_eid)
    , m_local_fid(local_fid)
    , m_hash(hash)
{}
} // namespace wmtk
#include "Tuple.hxx"
