#pragma once

#include "PrimitiveType.hpp"


namespace wmtk {

class Mesh;
class PointMesh;
class TriMesh;
class EdgeMesh;
class TetMesh;
namespace attribute {
template <typename T>
class TupleAccessor;
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
class MultiMeshManager;

class Tuple
{
private:
    // if Tuple is in 2d mesh m_global_cid is the global triangle id, and local_fid is -1
    // if Tuple is in 3d mesh m_global_cid is the global tetrahedron id
    int8_t m_local_vid = -1;
    int8_t m_local_eid = -1;
    int8_t m_local_fid = -1;
    int8_t m_hash = -1;
    int64_t m_global_cid = -1;

public:
    friend class Mesh;
    friend class PointMesh;
    friend class EdgeMesh;
    friend class TriMesh;
    friend class TetMesh;
    friend class MultiMeshManager;
    template <typename T>
    friend class attribute::TupleAccessor;
    friend class operations::Operation;
    friend class utils::TupleCellLessThan;
    friend class utils::TupleInspector;
    // friend int64_t Mesh::id(const Tuple& tuple, const PrimitiveType& type) const;
    // friend Mesh::is_ccw(const Tuple& tuple) const;
    // friend Mesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const;

    Tuple(int8_t local_vid, int8_t local_eid, int8_t local_fid, int64_t global_cid, int8_t hash);

    //         v2
    //       /    \.
    //  e1  /      \  e0
    //     v0 - - - v1
    //         e2

    Tuple();
    Tuple(const Tuple& other);
    Tuple(Tuple&& other);
    Tuple& operator=(const Tuple& other);
    Tuple& operator=(Tuple&& other);

    bool operator==(const Tuple& t) const;
    bool operator!=(const Tuple& t) const;
    bool operator<(const Tuple& t) const;
    // equality comparison but skips the hash
    bool same_ids(const Tuple& t) const;

    bool is_null() const;
    Tuple with_updated_hash(int64_t new_hash) const;
};
} // namespace wmtk
