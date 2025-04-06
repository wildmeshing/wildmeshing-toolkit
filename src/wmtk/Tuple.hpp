#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <tuple>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk {

/**
 * The `Tuple` is the basic navigation tool in our mesh data structure. It consists of the global
 * cell ID, and of local IDs for any other `PrimitiveType`. We highest simplex dimension we can
 * represent is 3, i.e., tetrahedra. The cell id always represents the top simplex type. So, for a
 * `TriMesh`, the cell ID is the triangle ID, and the local face ID will remain empty (-1).
 */
class Tuple
{
private:
    // if Tuple is in 2d mesh m_global_cid is the global triangle id, and local_fid is -1
    // if Tuple is in 3d mesh m_global_cid is the global tetrahedron id
    int64_t m_global_cid = -1;
    int8_t m_local_vid = -1;
    int8_t m_local_eid = -1;
    int8_t m_local_fid = -1;
#ifdef _WIN32
    std::array<int8_t, 5> m_pad = {{0, 0, 0, 0, 0}}; // align Tuple with 2*int64_t
#endif

public:
    Tuple(int8_t local_vid, int8_t local_eid, int8_t local_fid, int64_t global_cid);

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

    int64_t global_cid() const;
    int8_t local_vid() const;
    int8_t local_eid() const;
    int8_t local_fid() const;

    int8_t local_id(const PrimitiveType pt) const;

    std::string as_string() const;
    explicit operator std::string() const;

    friend std::ostream& operator<<(std::ostream& os, const Tuple& t);
};

std::ostream& operator<<(std::ostream& os, const Tuple& t);

} // namespace wmtk
#include "Tuple.hxx"
