#pragma once

#include <tbb/enumerable_thread_specific.h>
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <tuple>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {

class Tuple
{
private:
    size_t m_local_vid = -1;
    size_t m_local_eid = -1;
    size_t m_local_fid = -1;
    size_t m_global_cid = -1;
    size_t m_hash = -1;

    void update_hash(const Mesh& m);

public:
    Tuple(size_t local_vid, size_t local_eid, size_t local_fid, size_t global_cid, const Mesh& m)
        : m_local_vid(local_vid)
        , m_local_eid(local_eid)
        , m_local_fid(local_fid)
        , m_global_cid(global_cid)
    {
        update_hash(m);
    }
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

    /**
     * @brief return the global id of the Tuple of the given dimension
     *
     * @param m
     * @param dimension     d-0 -> vertex
                            d-1 -> edge
                            d-2 -> face
                            d-3 -> tetrahedron
     * @return size_t id of the entity
     */
    size_t id(const Mesh& m, const int dimension) const;
    /**
     * @brief switch in place the orientation of the Tuple of the given dimension
     *
     * @param m
     * @param dimension d-0 -> switch vertex
                        d-1 -> switch edge
                        d-2 -> switch face
                        d-3 -> switch tetrahedron
     */
    void sw(const Mesh& m, const int dimension) const;
    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true
     * @return false
     */
    bool is_valid(const Mesh& m) const;
    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    bool is_ccw(const Mesh& m) const;
};
} // namespace wmtk
