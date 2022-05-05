#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <memory>
#include <wmtk/utils/Logger.hpp>

#include <tbb/concurrent_vector.h>
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <vector>
#include "wmtk/AttributeCollection.hpp"

namespace wmtk {

class TriMesh
{
public:
    // Cell Tuple Navigator
    class Tuple
    {
    private:
        size_t m_vid;
        size_t m_eid;
        size_t m_fid;
        size_t m_hash;

        void update_hash(const TriMesh& m) { m_hash = m.m_tri_connectivity[m_fid].hash; }

    public:
        void print_info() { logger().trace("tuple: {} {} {}", m_vid, m_eid, m_fid); }

        //        v2         *
        //      /    \       *
        // e1  /      \  e0  *
        //    v0 - - - v1    *
        //        e2         *
        /**
         * Construct a new Tuple object with global vertex/triangle index and local edge index
         *
         * @param vid vertex id
         * @param eid edge id (local)
         * @param fid face id
         */
        Tuple() {}
        Tuple(size_t vid, size_t eid, size_t fid, const TriMesh& m)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
        {
            update_hash(m);
        }


        /**
         * returns global vertex id.
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         */
        inline size_t vid(const TriMesh&) const { return m_vid; }

        /**
         * returns a global unique face id
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         */
        inline size_t fid(const TriMesh&) const { return m_fid; }


        /**
         * returns a global unique edge id
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         * @note The global id may not be consecutive. The edges are undirected and different tetra
         * share the same edge.
         */
        inline size_t eid(const TriMesh& m) const
        {
            if (switch_face(m).has_value()) {
                size_t fid2 = switch_face(m)->fid(m);
                size_t min_fid = std::min(m_fid, fid2);
                if (min_fid == fid2) {
                    int i = m.m_tri_connectivity[fid2].find(m_vid);
                    int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
                    return min_fid * 3 + 3 - i - j;
                }
            }
            return m_fid * 3 + m_eid;
        }

        /**
         * Switch operation. See (URL-TO-DOCUMENT) for explaination.
         *
         * @param m
         * @return Tuple another Tuple that share the same face, edge, but different vertex.
         */
        Tuple switch_vertex(const TriMesh& m) const;

        Tuple switch_edge(const TriMesh& m) const;

        /**
         * Switch operation for the adjacent triangle
         *
         * @param m Mesh
         * @return Tuple for the edge-adjacent triangle, sharing same edge, and vertex.
         * @return nullopt if the Tuple of the switch goes off the boundary.
         */
        std::optional<Tuple> switch_face(const TriMesh& m) const;

        bool is_valid(const TriMesh& m) const;

        /**
         * Positively oriented 3 vertices (represented by Tuples) in a tri.
         * @return std::array<Tuple, 3> each tuple owns a different vertex.
         */
        std::array<Tuple, 3> oriented_tri_vertices(const TriMesh& m) const;
        friend bool operator<(const Tuple& a, const Tuple& t)
        {
            return (
                std::tie(a.m_vid, a.m_eid, a.m_fid, a.m_hash) <
                std::tie(t.m_vid, t.m_eid, t.m_fid, t.m_hash));
        }
    };

    /**
     * (internal use) Maintains a list of triangles connected to the given vertex, and a flag to
     * mark removal.
     *
     */
    class VertexConnectivity
    {
    public:
        std::vector<size_t> m_conn_tris;
        bool m_is_removed = false;

        inline size_t& operator[](const size_t index)
        {
            assert(index < m_conn_tris.size());
            return m_conn_tris[index];
        }

        inline size_t operator[](const size_t index) const
        {
            assert(index < m_conn_tris.size());
            return m_conn_tris[index];
        }
    };

    /**
     * (internal use) Maintains a list of vertices of the given tiangle
     *
     */
    class TriangleConnectivity
    {
    public:
        std::array<size_t, 3> m_indices;
        bool m_is_removed = false;
        size_t hash = 0;

        inline size_t& operator[](size_t index)
        {
            assert(index < 3);
            return m_indices[index];
        }

        inline size_t operator[](size_t index) const
        {
            assert(index < 3);
            return m_indices[index];
        }

        inline int find(int v_id) const
        {
            for (int j = 0; j < 3; j++) {
                if (v_id == m_indices[j]) return j;
            }
            return -1;
        }
    };

    TriMesh()
    {
        p_vertex_attrs = &vertex_attrs;
        p_edge_attrs = &edge_attrs;
        p_face_attrs = &face_attrs;
    }
    virtual ~TriMesh() {}

    void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris);

    /**
     * Generate a vector of Tuples from global vertex index and __local__ edge index
     * @note each vertex generate tuple that has the fid to be the smallest among
     * connected triangles' fid local vid to be in the same order as thier indices
     * in the m_conn_tris local eid assigned counter clockwise as in the ilustrated
     * example
     * @return vector of Tuples
     */
    std::vector<Tuple> get_vertices() const;

    /**
     * Generate a vector of Tuples from global face index
     * @note vid is the first of the m_idices
     * local eid assigned counter clockwise as in the ilustrated example
     * @return vector of Tuples
     */
    std::vector<Tuple> get_faces() const;

    /**
     * Generate a vector of Tuples for each edge
     * @note ensures the fid assigned is the smallest between faces adjacent to the
     * edge
     * @return vector of Tuples
     */
    std::vector<Tuple> get_edges() const;

    // uing the local vid to init an edge.
    // tuple is refereing to vid1, with the other endpoint at vid2.
    Tuple init_from_edge(size_t vid1, size_t vid2, size_t fid) const
    {
        auto a = m_tri_connectivity[fid].find(vid1);
        auto b = m_tri_connectivity[fid].find(vid2);
        assert(a != -1 && b != -1);
        // 0,1 - >2, 1,2-> 0, 0,2->1
        return Tuple(vid1, 3 - (a + b), fid, *this);
    }

    template <typename T>
    using vector = tbb::concurrent_vector<T>;

public:
    AbstractAttributeContainer *p_vertex_attrs, *p_edge_attrs, *p_face_attrs;
    AbstractAttributeContainer vertex_attrs, edge_attrs, face_attrs;

    // write a file has boundary vertices correspondences
    Eigen::MatrixXi bnd_table;
    void get_boundary_map(Eigen::VectorXi SVI); // takes the mapping form the after remove_duplicate
                                                // to the original input
private:
    vector<VertexConnectivity> m_vertex_connectivity;
    vector<TriangleConnectivity> m_tri_connectivity;

    size_t get_next_empty_slot_t();
    size_t get_next_empty_slot_v();

protected:
    virtual bool invariants(const std::vector<Tuple>&) { return true; }
    virtual bool split_edge_before(const Tuple& t) { return true; }
    virtual bool split_edge_after(const Tuple& t) { return true; }


    virtual bool collapse_edge_before(const Tuple& t)
    {
        if (check_link_condition(t)) return true;
        return false;
    }
    virtual bool collapse_edge_after(const Tuple& t) { return true; }
    virtual bool swap_edge_after(const Tuple& t) { return true; }
    virtual bool swap_edge_before(const Tuple& t)
    {
        if (!t.switch_face(*this).has_value())
            return false; // can't swap on boundary edgereturn true;
        // when swap edge between v1, v2, there can't exist edge between v3, v4
        // already
        size_t v4 =
            ((t.switch_face(*this).value()).switch_edge(*this)).switch_vertex(*this).vid(*this);
        size_t v3 = ((t.switch_edge(*this)).switch_vertex(*this)).vid(*this);
        if (!set_intersection(
                 m_vertex_connectivity[v4].m_conn_tris,
                 m_vertex_connectivity[v3].m_conn_tris)
                 .empty())
            return false;
        return true;
    }
    virtual bool smooth_before(const Tuple& t) { return true; }
    virtual bool smooth_after(const Tuple& t) { return true; }
    virtual void resize_mutex(size_t v){}; // tempoarary hack


public:
    size_t tri_capacity() const { return m_tri_connectivity.size(); }
    size_t vert_capacity() const { return m_vertex_connectivity.size(); }

    void consolidate_mesh(bool bnd_output = false);

    Tuple switch_vertex(const Tuple& t) const { return t.switch_vertex(*this); }
    Tuple switch_edge(const Tuple& t) const { return t.switch_edge(*this); }
    std::optional<Tuple> switch_face(const Tuple& t) const { return t.switch_face(*this); }

    bool check_link_condition(const Tuple& t) const;
    bool check_mesh_connectivity_validity() const;
    bool check_internal_link_condition(const Tuple& t) const;
    bool check_edge_manifold() const;
    bool is_boundary_edge(const TriMesh::Tuple& t) const
    {
        if (!t.switch_face(*this).has_value()) return true;
        return false;
    }

    /**
     * Split an edge
     *
     * @param t Input Tuple for the edge to split.
     * @param[out] new_edges a vector of Tuples for all the edges from the newly
     * introduced triangle
     * @return if split succeed
     */
    bool split_edge(const Tuple& t, std::vector<Tuple>& new_t);
    bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_t);
    bool swap_edge(const Tuple& t, std::vector<Tuple>& new_t);
    bool smooth_vertex(const Tuple& t);

    /**
     * @brief Get the one ring tris for a vertex
     *
     * @param t tuple pointing to a vertex
     * @return one-ring
     */
    std::vector<Tuple> get_one_ring_tris_for_vertex(const Tuple& t) const;

    /**
     * @brief Get the one ring edges for a vertex, edges are the incident edges
     *
     * @param t tuple pointing to a vertex
     * @return one-ring
     */
    std::vector<Tuple> get_one_ring_edges_for_vertex(const Tuple& t) const;

    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return incident vertices
     */
    std::array<Tuple, 3> oriented_tri_vertices(const Tuple& t) const;


    Tuple tuple_from_tri(size_t fid) const
    {
        auto vid = m_tri_connectivity[fid][0];
        return Tuple(vid, 1, fid, *this);
    }
    Tuple tuple_from_vertex(size_t vid) const
    {
        auto fid = m_vertex_connectivity[vid][0];
        auto eid = m_tri_connectivity[fid].find(vid);
        return Tuple(vid, (eid + 1) % 3, fid, *this);
    }
    Tuple tuple_from_edge(size_t fid, size_t local_eid) const
    {
        auto vid = m_tri_connectivity[fid][(local_eid + 1) % 3];
        return Tuple(vid, local_eid, fid, *this);
    }

private:
    void start_protect_attributes()
    {
        p_vertex_attrs->begin_protect();
        p_edge_attrs->begin_protect();
        p_face_attrs->begin_protect();
    }

    void release_protect_attributes()
    {
        p_vertex_attrs->end_protect();
        p_edge_attrs->end_protect();
        p_face_attrs->end_protect();
    }

    void rollback_protected_attributes()
    {
        p_vertex_attrs->rollback();
        p_edge_attrs->rollback();
        p_face_attrs->rollback();
    }
};

} // namespace wmtk
