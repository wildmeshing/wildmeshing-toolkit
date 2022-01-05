
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <vector>

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

        //        v2
        //      /    \
		// e1  /      \  e0
        //    v0 - - - v1
        //        e2
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


        inline size_t get_vid() const { return m_vid; }
        inline size_t get_eid() const
        {
            return m_fid * 3 + m_eid;
        } // this is unique eid. each edge is repeated twice?
        inline size_t get_fid() const { return m_fid; }

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

        bool is_valid(const TriMesh& m) const
        {
            if (m.m_vertex_connectivity[m_vid].m_is_removed ||
                m.m_tri_connectivity[m_fid].m_is_removed)
                return false;

            // Condition 3: tuple m_hash check
            if (m_hash != m.m_tri_connectivity[m_fid].hash) return false;

#ifndef NDEBUG
            //  Condition 0: Elements exist
            assert(m_vid < m.m_vertex_connectivity.size());
            assert(m_eid <= 2);
            assert(m_fid <= m.m_tri_connectivity.size());

            // Condition 1: tid and vid are consistent
            const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
            assert(lvid == 0 || lvid == 1 || lvid == 2);

            // Condition 2: eid is valid
            const int v0 = m.m_tri_connectivity[m_fid][0];
            const int v1 = m.m_tri_connectivity[m_fid][1];
            const int v2 = m.m_tri_connectivity[m_fid][2];
            switch (m_eid) {
            case 0: assert(m_vid == v1 || m_vid == v2); break;
            case 1: assert(m_vid == v0 || m_vid == v2); break;
            case 2: assert(m_vid == v0 || m_vid == v1); break;
            }
#endif

            return true;
        }

        /**
         * Positively oriented 3 vertices (represented by Tuples) in a tri.
         * @return std::array<Tuple, 3> each tuple owns a different vertex.
         */
        std::array<Tuple, 3> oriented_tri_vertices(const TriMesh& m) const
        {
            std::array<Tuple, 3> vs;
            for (int j = 0; j < 3; j++) {
                vs[j].m_vid = m.m_tri_connectivity[m_fid][j];
                vs[j].m_eid = (j + 2) % 3;
                vs[j].m_fid = m_fid;
            }
            return vs;
        }

        size_t get_vertex_attribute_id(const TriMesh& m);
        size_t get_edge_attribute_id(const TriMesh& m);
        size_t get_face_attribute_id(const TriMesh& m);
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
            assert(index >= 0 && index < m_conn_tris.size());
            return m_conn_tris[index];
        }

        inline size_t operator[](const size_t index) const
        {
            assert(index >= 0 && index < m_conn_tris.size());
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
        size_t hash;

        inline size_t& operator[](size_t index)
        {
            assert(index >= 0 && index < 3);
            return m_indices[index];
        }

        inline size_t operator[](size_t index) const
        {
            assert(index >= 0 && index < 3);
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

    TriMesh() {}
    virtual ~TriMesh() {}

    inline void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
    {
        m_vertex_connectivity.resize(n_vertices);
        m_tri_connectivity.resize(tris.size());
        size_t hash_cnt = 0;
        for (int i = 0; i < tris.size(); i++) {
            m_tri_connectivity[i].m_indices = tris[i];
            m_tri_connectivity[i].hash = hash_cnt;
            hash_cnt++;
            for (int j = 0; j < 3; j++) m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
        }
    }

    /**
     * Generate a vector of Tuples from global vertex index and __local__ edge index
     * @note each vertex generate tuple that has the fid to be the smallest among connected
     * triangles' fid local vid to be in the same order as thier indices in the m_conn_tris local
     * eid assigned counter clockwise as in the ilustrated example
     * @return vector of Tuples
     */
    std::vector<Tuple> get_vertices() const
    {
        const TriMesh& m = *this;
        const size_t n_vertices = m_vertex_connectivity.size();
        std::vector<Tuple> all_vertices_tuples;
        all_vertices_tuples.resize(n_vertices);
        for (size_t i = 0; i < n_vertices; i++) {
            const std::vector<size_t>& v_conn_fids = m_vertex_connectivity[i].m_conn_tris;
            size_t fid = *min_element(v_conn_fids.begin(), v_conn_fids.end());

            // get the 3 vid
            const std::array<size_t, 3> f_conn_verts = m_tri_connectivity[fid].m_indices;
            assert(i == f_conn_verts[0] || i == f_conn_verts[1] || i == f_conn_verts[2]);
            size_t eid;
            // eid is the same as the lvid
            if (i == f_conn_verts[0]) eid = 2;
            if (i == f_conn_verts[1])
                eid = 0;
            else
                eid = 1;

            Tuple v_tuple = Tuple(i, eid, fid, m);
            assert(v_tuple.is_valid(m));
            all_vertices_tuples[i] = v_tuple;
        }
        return all_vertices_tuples;
    }

    /**
     * Generate a vector of Tuples from global face index
     * @note vid is the first of the m_idices
     * local eid assigned counter clockwise as in the ilustrated example
     * @return vector of Tuples
     */
    std::vector<Tuple> get_faces() const
    {
        const TriMesh& m = *this;
        std::vector<Tuple> all_faces_tuples;
        all_faces_tuples.resize(m.m_tri_connectivity.size());
        for (size_t i = 0; i < m.m_tri_connectivity.size(); i++) {
            // get the 3 vid
            const std::array<size_t, 3>& f_conn_verts = m.m_tri_connectivity[i].m_indices;
            size_t vid = f_conn_verts[0];
            Tuple f_tuple = Tuple(vid, 2, i, m);
            assert(f_tuple.is_valid(m));
            all_faces_tuples[i] = f_tuple;
        }
        return all_faces_tuples;
    }

    /**
     * Generate a vector of Tuples for each edge
     * @note ensures the fid assigned is the smallest between faces adjacent to the edge
     * @return vector of Tuples
     */
    std::vector<Tuple> get_edges() const
    {
        const TriMesh& m = *this;
        std::vector<Tuple> all_edges_tuples;
        all_edges_tuples.reserve(m.m_tri_connectivity.size() * 3 / 2);
        for (size_t i = 0; i < m.m_tri_connectivity.size(); i++) {
            for (int j = 0; j < 3; j++) {
                size_t vid = m.m_tri_connectivity[i].m_indices[j];
                size_t eid = (j + 2) % 3;
                Tuple e_tuple = Tuple(vid, eid, i, m);
                assert(e_tuple.is_valid(m));
                Tuple e_tuple2 = e_tuple.switch_face(m).value_or(e_tuple);
                assert(e_tuple2.is_valid(m));
                // return itself if it is a boundary triangle
                size_t fid2 = e_tuple2.get_fid();
                if (fid2 < i)
                    continue;
                else
                    all_edges_tuples.push_back(e_tuple);
            }
        }
        return all_edges_tuples;
    }

private:
    std::vector<VertexConnectivity> m_vertex_connectivity;
    std::vector<TriangleConnectivity> m_tri_connectivity;

    size_t get_next_empty_slot_t()
    {
        m_tri_connectivity.emplace_back();
        resize_attributes(m_vertex_connectivity.size(),m_tri_connectivity.size());
        return m_tri_connectivity.size() - 1;
    }
    size_t get_next_empty_slot_v()
    {
        m_vertex_connectivity.emplace_back();
        resize_attributes(m_vertex_connectivity.size(),m_tri_connectivity.size());
        return m_vertex_connectivity.size() - 1;
    }

protected:
    virtual bool split_before(const Tuple& t) { return true; }
    virtual bool split_after(const Tuple& t) { return true; }
    // check link, check if it's the last edge
    virtual bool collapse_before(const Tuple& t)
    {
        // DP: these should be empty
        auto v1_conn_tris = m_vertex_connectivity[t.get_vid()].m_conn_tris;
        auto v2_conn_tris = m_vertex_connectivity[switch_vertex(t).get_vid()].m_conn_tris;

        size_t fid1 = t.get_fid();
        size_t fid2 = switch_face(t).value_or(t).get_fid();

        vector_erase(v1_conn_tris, fid1);
        vector_erase(v1_conn_tris, fid2);
        vector_erase(v2_conn_tris, fid1);
        vector_erase(v2_conn_tris, fid2);
        // check if this is a tet (the final state of collapsing for a closed mesh)
        // if (v1_conn_tris.size() == 1 && v2_conn_tris.size() == 1) {
        //     auto m1_indices = m_tri_connectivity[v1_conn_tris[0]].m_indices;
        //     auto m2_indices = m_tri_connectivity[v2_conn_tris[0]].m_indices;
        //     int cnt = 0;

        //     if (intersection.size() == 2) return false;
        // }

        if (check_link_condition(t) && (v1_conn_tris.size() + v2_conn_tris.size() > 0)) return true;
        return false;
    }
    virtual bool collapse_after(const Tuple& t)
    {
        if (check_mesh_connectivity_validity() && t.is_valid(*this)) return true;
        return false;
    }

    virtual void resize_attributes(size_t v, size_t t) {}

public:
    size_t n_triangles() const { return m_tri_connectivity.size(); }
    size_t n_vertices() const { return m_vertex_connectivity.size(); }

    // just for testing
    std::vector<TriangleConnectivity> get_m_tri_connectivity() const { return m_tri_connectivity; }
    // just for testing
    std::vector<VertexConnectivity> get_m_vertex_connectivity() const
    {
        return m_vertex_connectivity;
    }


    Tuple switch_vertex(const Tuple& t) const { return t.switch_vertex(*this); }
    Tuple switch_edge(const Tuple& t) const { return t.switch_edge(*this); }
    std::optional<Tuple> switch_face(const Tuple& t) const { return t.switch_face(*this); }

    bool check_link_condition(const Tuple& t) const; // DP: should be private
    bool check_mesh_connectivity_validity() const; // DP: should be private
    std::vector<size_t> compact();

    /**
     * Split an edge
     *
     * @param t Input Tuple for the edge to split.
     * @param[out] new_edges a vector of Tuples for all the edges from the newly introduced
     * triangle
     * @return if split succeed
     */
    bool split_edge(const Tuple& t, Tuple& new_t);
    bool collapse_edge(const Tuple& t, Tuple& new_t);
    void swap_edge(const Tuple& t, int type);

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
    std::vector<Tuple> get_oriented_vertices_for_tri(const Tuple& t) const;
};

} // namespace wmtk
