//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>
#include <wmtk/Logger.hpp>

#include <array>
#include <cassert>
#include <map>
#include <optional>
#include <queue>
#include <vector>

namespace wmtk {
class TetMesh
{
public:
    // Cell Tuple Navigator
    class Tuple
    {
    private:
        static constexpr std::array<std::array<int, 2>, 6> m_local_edges = {
            {{{0, 1}}, {{1, 2}}, {{2, 0}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}}; // local edges within a
                                                                           // tet
        static constexpr std::array<int, 6> m_map_vertex2edge = {{0, 0, 1, 3}};
        static constexpr std::array<int, 6> m_map_edge2face = {{0, 0, 0, 1, 2, 1}};
        static constexpr std::array<std::array<int, 3>, 6> m_local_faces = {
            {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 1}}, {{3, 2, 1}}}};
        static constexpr std::array<std::array<int, 3>, 6> m_local_edges_in_a_face = {
            {{{0, 1, 2}}, {{2, 5, 3}}, {{3, 4, 0}}, {{5, 1, 4}}}};

        size_t m_vid;
        size_t m_eid;
        size_t m_fid;
        size_t m_tid;

        int m_timestamp = 0;

    public:
        static Tuple init_from_edge(const TetMesh& m, int tid, int local_eid)
        {
            int vid = m.m_tet_connectivity[tid][m_local_edges[local_eid][0]];
            int fid = m_map_edge2face[local_eid];
            return Tuple(vid, local_eid, fid, tid);
        }

        static Tuple init_from_vertex(const TetMesh& m, int vid)
        {
            int tid = m.m_vertex_connectivity[vid].m_conn_tets[0];
            int j = m.m_tet_connectivity[tid].find(vid);
            int eid = m_map_vertex2edge[j];
            int fid = m_map_edge2face[eid];
            return Tuple(vid, eid, fid, tid);
        }

    public:
        bool is_valid(const TetMesh& m) const
        {
            if (m.m_vertex_connectivity[m_vid].m_is_removed ||
                m.m_tet_connectivity[m_tid].m_is_removed)
                return false;
            return true;
        }

        void update_version_number(const TetMesh& m)
        {
            assert(m_timestamp >= m.m_tet_connectivity[m_tid].timestamp);
            m_timestamp = m.m_tet_connectivity[m_tid].timestamp;
        }

        int get_version_number() { return m_timestamp; }

        bool is_version_number_valid(const TetMesh& m) const
        {
            if (m_timestamp != m.m_tet_connectivity[m_tid].timestamp) return false;
            return true;
        }

        void print_info() { logger().trace("tuple: {} {} {} {}", m_vid, m_eid, m_fid, m_tid); }

        Tuple() {}

        Tuple(size_t vid, size_t eid, size_t fid, size_t tid)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
            , m_tid(tid)
        {} // DP: the counter should be initialized here?

        size_t vid() const { return m_vid; } // update eid and fid

        size_t eid(const TetMesh& m) const
        {
            int v1_id = m.m_tet_connectivity[m_tid][m_local_edges[m_eid][0]];
            int v2_id = m.m_tet_connectivity[m_tid][m_local_edges[m_eid][1]];
            auto n12_t_ids = set_intersection(m.m_vertex_connectivity[v1_id].m_conn_tets,
                                              m.m_vertex_connectivity[v2_id].m_conn_tets);
            int tid = *std::min_element(n12_t_ids.begin(), n12_t_ids.end());
            for(int j=0;j<6;j++){
                int tmp_v1_id = m.m_tet_connectivity[tid][m_local_edges[j][0]];
                int tmp_v2_id = m.m_tet_connectivity[tid][m_local_edges[j][1]];
                if(tmp_v1_id == v1_id && tmp_v2_id == v2_id
                    ||tmp_v1_id == v2_id && tmp_v2_id == v1_id)
                    return tid * 6 + j;
            }
            throw std::runtime_error("Tuple::eid() error");
        }

        size_t fid(const TetMesh& m) const
        {
            std::array<size_t, 3> v_ids = {
                {m.m_tet_connectivity[m_tid][m_local_faces[m_fid][0]],
                 m.m_tet_connectivity[m_tid][m_local_faces[m_fid][1]],
                 m.m_tet_connectivity[m_tid][m_local_faces[m_fid][2]]}};
            auto tmp = set_intersection(
                m.m_vertex_connectivity[v_ids[0]].m_conn_tets,
                m.m_vertex_connectivity[v_ids[1]].m_conn_tets);
            auto n12_t_ids = set_intersection(tmp, m.m_vertex_connectivity[v_ids[2]].m_conn_tets);
            if (n12_t_ids.size() == 1) {
                return m_tid * 4 + m_fid;
            }

            std::sort(v_ids.begin(), v_ids.end());
            int tid = *std::min_element(n12_t_ids.begin(), n12_t_ids.end());
            for (int j = 0; j < 4; j++) {
                std::array<size_t, 3> tmp_v_ids = {
                    {m.m_tet_connectivity[tid][m_local_faces[j][0]],
                     m.m_tet_connectivity[tid][m_local_faces[j][1]],
                     m.m_tet_connectivity[tid][m_local_faces[j][2]]}};
                std::sort(tmp_v_ids.begin(), tmp_v_ids.end());
                if (tmp_v_ids == v_ids) return tid * 4 + j;
            }

            throw std::runtime_error("Tuple::fid() error");
        }

        size_t tid() const { return m_tid; }

        Tuple switch_vertex(const TetMesh& m) const
        {
            Tuple loc = *this;
            int l_vid1 = m_local_edges[m_eid][0];
            int l_vid2 = m_local_edges[m_eid][1];
            loc.m_vid = m.m_tet_connectivity[m_tid][l_vid1] == m_vid
                            ? m.m_tet_connectivity[m_tid][l_vid2]
                            : m.m_tet_connectivity[m_tid][l_vid1];

            return loc;
        } // along edge

        Tuple switch_edge(const TetMesh& m) const
        {
            Tuple loc = *this;
            for (int j = 0; j < 3; j++) {
                if (m_local_edges_in_a_face[m_fid][j] == m_eid) {
                    loc.m_eid = m_local_edges_in_a_face[m_fid][(j + 1) % 3];
                    return loc;
                }
            }
            assert("switch edge failed");
            return loc;
        }

        Tuple switch_face(const TetMesh& m) const
        {
            Tuple loc = *this;
            int l_v1_id = m_local_edges[m_eid][0];
            int l_v2_id = m_local_edges[m_eid][1];
            for (int j = 0; j < 4; j++) {
                if (j == m_fid) continue;
                int cnt = 0;
                for (int k = 0; k < 3; k++) {
                    if (m_local_faces[j][k] == l_v1_id || m_local_faces[j][k] == l_v2_id) cnt++;
                    if (cnt == 2) {
                        loc.m_fid = j;
                        return loc;
                    }
                }
            }
            assert("switch face failed");
            return loc;
        }

        std::optional<Tuple> switch_tetrahedron(const TetMesh& m) const
        {
            // eid and fid are local, so they will be changed after switch tets
            int v1_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][0]];
            int v2_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][1]];
            int v3_id = m.m_tet_connectivity[m_tid][m_local_faces[m_fid][2]];
            auto tmp = set_intersection(
                m.m_vertex_connectivity[v1_id].m_conn_tets,
                m.m_vertex_connectivity[v2_id].m_conn_tets);
            auto n123_tids = set_intersection(tmp, m.m_vertex_connectivity[v3_id].m_conn_tets);
            if (n123_tids.size() == 1)
                return {};
            else {
                Tuple loc = *this;
                loc.m_tid = n123_tids[0] == m_tid ? n123_tids[1] : n123_tids[0];
                int j = m.m_tet_connectivity[loc.m_tid].find(loc.m_vid);
                loc.m_eid = m_map_vertex2edge[j];
                loc.m_fid = m_map_vertex2edge[loc.m_eid];
                return loc;
            }
        }

        std::array<Tuple, 4> oriented_tet_vertices(const TetMesh& m) const
        {
            std::array<Tuple, 4> vs;
            for (int j = 0; j < 4; j++) {
                vs[j].m_vid = m.m_tet_connectivity[m_tid][j];
                vs[j].m_eid = m_map_vertex2edge[j];
                vs[j].m_fid = m_map_edge2face[vs[j].m_eid];
                vs[j].m_tid = m_tid;
            }
            return vs;
        }
    };

    class VertexConnectivity
    {
    public:
        std::vector<size_t> m_conn_tets; // todo: always keep it sorted
        bool m_is_removed = false;

        size_t& operator[](const size_t index)
        {
            assert(index >= 0 && index < m_conn_tets.size());
            return m_conn_tets[index];
        }

        size_t operator[](const size_t index) const
        {
            assert(index >= 0 && index < m_conn_tets.size());
            return m_conn_tets[index];
        }
    };

    class TetrahedronConnectivity
    {
    public:
        std::array<size_t, 4> m_indices;
        bool m_is_removed = false;

        int timestamp = 0;

        void set_version_number(int version) { timestamp = version; }
        int get_version_number() { return timestamp; }

        size_t& operator[](size_t index)
        {
            assert(index >= 0 && index < 4);
            return m_indices[index];
        }

        size_t operator[](size_t index) const
        {
            assert(index >= 0 && index < 4);
            return m_indices[index];
        }

        int find(int v_id) const
        {
            for (int j = 0; j < 4; j++) {
                if (v_id == m_indices[j]) return j;
            }
            return -1;
        }
    };

    TetMesh() {}
    virtual ~TetMesh() {}

    void init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets);

    bool split_edge(const Tuple& t, std::vector<Tuple>& new_edges);
    bool collapse_edge(const Tuple& t, std::vector<Tuple>& new_edges);
    void swap_edge(const Tuple& t, int type);

    void
    compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices

    void reset_timestamp()
    {
        m_timestamp = 0;
        for (auto& t : m_tet_connectivity) t.timestamp = 0;
    }

    std::vector<Tuple> get_edges() const;

    size_t n_tets() const { return m_tet_connectivity.size(); }
    size_t v_id(int tid, int lvid) const { return m_tet_connectivity[tid][lvid]; }

private:
    // Stores the connectivity of the mesh
    std::vector<VertexConnectivity> m_vertex_connectivity;
    std::vector<TetrahedronConnectivity> m_tet_connectivity;

    int m_t_empty_slot = 0;
    int m_v_empty_slot = 0;
    int find_next_empty_slot_t();
    int find_next_empty_slot_v();

    int m_timestamp = 0;

protected:
    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    virtual bool split_before(const Tuple& t) { return true; } // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    virtual bool split_after(const std::vector<Tuple>& locs) { return true; } // check tet condition

    //// Collapse the edge in the tuple
    // Checks if the collapse should be performed or not (user controlled)
    virtual bool collapse_before(const Tuple& t) { return true; }
    // If it returns false then the operation is undone (the tuple indexes a vertex and tet that
    // survived)
    virtual bool collapse_after(const std::vector<Tuple>& locs) { return true; }
    // todo: quality, inversion, envelope: change v1 pos before this, only need to change partial
    // attributes

    //        //// Swap the edge in the tuple
    //        // Checks if the swapping should be performed or not (user controlled)
    //        virtual bool swapping_before(const Tuple &t) { return true; }
    //        // If it returns false then the operation is undone (the tuple indexes TODO)
    //        virtual bool swapping_after(const Tuple &t) { return true; }
    //        //quality, inversion
    //
    // Invariants that are called on all the new or modified elements after an operation is
    // performed
    virtual bool vertex_invariant(const Tuple& t) { return true; }
    virtual bool edge_invariant(const Tuple& t) { return true; }
    virtual bool face_invariant(const Tuple& t) { return true; }
    virtual bool tetrahedron_invariant(const Tuple& t) { return true; }

    virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) {}

public:
    Tuple switch_vertex(const Tuple& t) const { return t.switch_vertex(*this); }
    Tuple switch_edge(const Tuple& t) const { return t.switch_edge(*this); }
    Tuple switch_face(const Tuple& t) const { return t.switch_face(*this); }
    std::optional<Tuple> switch_tetrahedron(const Tuple& t) const
    {
        return t.switch_tetrahedron(*this);
    }

    Tuple tuple_from_edge(int tid, int local_eid) const
    {
        return Tuple::init_from_edge(*this, tid, local_eid);
    }
};

} // namespace wmtk
