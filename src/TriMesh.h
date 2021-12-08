//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>
#include <wmtk/Logger.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
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
        size_t m_version_number;

    public:
        void print_info() { logger().trace("tuple: {} {} {}", m_vid, m_eid, m_fid); }

        void update_version_number(const TriMesh& m)
        {
            m_version_number = m.m_tri_connectivity[m_fid].version_number;
        }
        //        v2
        //      /    \ 
		// e1  /      \  e0
        //    v0 - - - v1
        //        e2

        Tuple() {}
        Tuple(size_t vid, size_t eid, size_t fid, const TriMesh& m)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
        {
            update_version_number(m);
        }


        inline size_t get_vid() const { return m_vid; }
        inline size_t get_eid() const
        {
            return m_fid * 3 + m_eid;
        } // this is unique eid. each edge is repeated twice?
        inline size_t get_fid() const { return m_fid; }

        const Tuple switch_vertex(const TriMesh& m)
        {
            assert(is_valid(m));

            const int v0 = m.m_tri_connectivity[m_fid][0];
            const int v1 = m.m_tri_connectivity[m_fid][1];
            const int v2 = m.m_tri_connectivity[m_fid][2];

            Tuple loc = *this;
            switch (m_eid) {
            case 0:
                assert(m_vid == v1 || m_vid == v2);
                loc.m_vid = m_vid == v1 ? v2 : v1;
                break;
            case 1:
                assert(m_vid == v0 || m_vid == v2);
                loc.m_vid = m_vid == v0 ? v2 : v0;
                break;
            case 2:
                assert(m_vid == v0 || m_vid == v1);
                loc.m_vid = m_vid == v0 ? v1 : v0;
                break;
            }

            assert(loc.is_valid(m));
            return loc;
        }

        const Tuple switch_edge(const TriMesh& m)
        {
            assert(is_valid(m));

            const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
            assert(lvid == 0 || lvid == 1 || lvid == 2);

            Tuple loc = *this;
            switch (lvid) {
            case 0:
                assert(m_eid == 1 || m_eid == 2);
                loc.m_eid = m_eid == 1 ? 2 : 1;
                break;
            case 1:
                assert(m_eid == 0 || m_eid == 2);
                loc.m_eid = m_eid == 0 ? 2 : 0;
                break;
            case 2:
                assert(m_eid == 0 || m_eid == 1);
                loc.m_eid = m_eid == 0 ? 1 : 0;
                break;
            }

            assert(loc.is_valid(m));
            return loc;
        }

        const std::optional<Tuple> switch_face(const TriMesh& m)
        {
            assert(is_valid(m));

            const size_t v0 = m_vid;
            const size_t v1 = this->switch_vertex(m).m_vid;

            // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
            std::vector<size_t> v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
            std::vector<size_t> v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

            std::sort(v0_fids.begin(), v0_fids.end());
            std::sort(v1_fids.begin(), v1_fids.end());
            std::vector<int> fids;
            std::set_intersection(
                v0_fids.begin(),
                v0_fids.end(),
                v1_fids.begin(),
                v1_fids.end(),
                std::back_inserter(fids)); // make sure this is correct
            assert(fids.size() == 1 || fids.size() == 2);

            if (fids.size() == 1) return {};

            Tuple loc = *this;

            // There is a triangle on the other side
            if (fids.size() == 2) {
                // Find the fid of the triangle on the other side
                size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];
                loc.m_fid = fid2;

                // Get sorted local indices of the two vertices in the new triangle
                size_t lv0_2 = m.m_tri_connectivity[fid2].find(v0);
                assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
                size_t lv1_2 = m.m_tri_connectivity[fid2].find(v1);
                assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

                if (lv0_2 > lv1_2) std::swap(lv0_2, lv1_2);

                // Assign the edge id depending on the table
                if (lv0_2 == 0 && lv1_2 == 1) {
                    loc.m_eid = 2;
                } else if (lv0_2 == 1 && lv1_2 == 2) {
                    loc.m_eid = 0;
                } else if (lv0_2 == 0 && lv1_2 == 2) {
                    loc.m_eid = 1;
                } else {
                    assert(false);
                }

                loc.m_version_number = m.m_tri_connectivity[loc.m_fid].version_number;
            }

            assert(loc.is_valid(m));
            return loc;
        }

        bool is_valid(const TriMesh& m)
        {
            // Condition 0: Elements exist
            if (m_vid < 0 || m_vid >= m.m_vertex_connectivity.size()) return false;

            if (m_eid < 0 || m_eid > 2) return false;

            if (m_fid < 0 || m_fid > m.m_tri_connectivity.size()) return false;

            // Condition 1: tid and vid are consistent
            const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
            if (!(lvid == 0 || lvid == 1 || lvid == 2)) return false;

            // Condition 2: eid is valid
            const int v0 = m.m_tri_connectivity[m_fid][0];
            const int v1 = m.m_tri_connectivity[m_fid][1];
            const int v2 = m.m_tri_connectivity[m_fid][2];

            switch (m_eid) {
            case 0:
                if (!(m_vid == v1 || m_vid == v2)) return false;
                break;
            case 1:
                if (!(m_vid == v0 || m_vid == v2)) return false;
                break;
            case 2:
                if (!(m_vid == v0 || m_vid == v1)) return false;
                break;
            }

            // Condition 3: the counter is up to date
            if (m_version_number != m.m_tri_connectivity[m_fid].version_number) return false;

            return true;
        }

        size_t get_vertex_attribute_id(const TriMesh& m);
        size_t get_edge_attribute_id(const TriMesh& m);
        size_t get_face_attribute_id(const TriMesh& m);
    };

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

    class TriangleConnectivity
    {
    public:
        std::array<size_t, 3> m_indices;
        bool m_is_removed = false;
        size_t version_number;

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
        for (int i = 0; i < tris.size(); i++) {
            m_tri_connectivity[i].m_indices = tris[i];
            for (int j = 0; j < 3; j++) m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
        }
    }

    // each vertex generate tuple that has the fid to be the smallest of connected triangles' fid
    // vertex local vid to be in the same order as thier indices in the m_conn_tris
    // eid assigned clockwise according to lvid in the illustration
    std::vector<Tuple> generate_tuples_from_vertices()
    {
        TriMesh m = *this;
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

    // given fid get connected vertices vid to be the first in the array
    // eid assigned clockwise
    std::vector<Tuple> generate_tuples_from_faces()
    {
        const TriMesh m = *this;
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

    // generate edge_tuple for each edge. the first scan of tuples will be 3 * #f
    // before pushing into the final all_edge_tuples check if the edge_tuple's fid is the smaller
    // one and only push in one (by callign the switch_edge)
    std::vector<Tuple> generate_tuples_from_edges()
    {
        const TriMesh m = *this;
        std::vector<Tuple> all_edges_tuples;
        all_edges_tuples.reserve(m.m_tri_connectivity.size() * 3 / 2);
        for (size_t i = 0; i < m.m_tri_connectivity.size(); i++) {
            for (int j = 0; j < 3; j++) {
                size_t vid = m.m_tri_connectivity[i].m_indices[j];
                size_t eid = (j + 2) % 3;
                Tuple e_tuple = Tuple(vid, eid, i, m);
                assert(e_tuple.is_valid(m));
                Tuple e_tuple2 = e_tuple.switch_face(m).value_or(
                    e_tuple); // return itdlrf if it is a boundary triangle
                size_t fid2 = e_tuple2.get_fid();
                if (fid2 < i)
                    continue;
                else
                    all_edges_tuples.push_back(e_tuple);
            }
        }
        return all_edges_tuples;
    }

    std::vector<Tuple> get_all_vertices();
    std::vector<Tuple> get_all_edges();
    std::vector<Tuple> get_all_tris();


private:
    // Stores the connectivity of the mesh
    std::vector<VertexConnectivity> m_vertex_connectivity;
    std::vector<TriangleConnectivity> m_tri_connectivity;


public:
    std::vector<VertexConnectivity> get_vertex_connectivity()
    {
        return this->m_vertex_connectivity;
    }
    std::vector<TriangleConnectivity> get_tri_connectivity() { return this->m_tri_connectivity; }

    Tuple switch_vertex(Tuple& t) const { return t.switch_vertex(*this); }
    Tuple switch_edge(Tuple& t) const { return t.switch_edge(*this); }
    std::optional<Tuple> switch_face(Tuple& t) const { return t.switch_face(*this); }

protected:
    //// Split the edge in the tuple
    // Checks if the split should be performed or not (user controlled)
    //virtual bool split_before(const Tuple &t) = 0; // check edge condition
    // This function computes the attributes for the added simplices
    // if it returns false then the operation is undone
    //virtual bool split_after(const Tuple &t) = 0; // check tet condition

    //        //// Collapse the edge in the tuple
    //        // Checks if the collapse should be performed or not (user controlled)
    //        virtual bool collapse_before(const Tuple &t) = 0;
    //        // If it returns false then the operation is undone (the tuple indexes a vertex and tet that survived)
    //        virtual bool collapse_after(const Tuple &t) = 0;
    //        //todo: quality, inversion, envelope: change v1 pos before this, only need to change partial attributes
    //
    //        //// Swap the edge in the tuple
    //        // Checks if the swapping should be performed or not (user controlled)
    //        virtual bool swapping_before(const Tuple &t) = 0;
    //        // If it returns false then the operation is undone (the tuple indexes TODO)
    //        virtual bool swapping_after(const Tuple &t) = 0;
    //        //quality, inversion
    //
    //        // Invariants that are called on all the new or modified elements after an operation is performed
    //        virtual bool VertexInvariant(const Tuple &t) = 0;
    //        virtual bool EdgeInvariant(const Tuple &t) = 0;
    //        virtual bool FaceInvariant(const Tuple &t) = 0;
    //        virtual bool TetrahedronInvariant(const Tuple &t) = 0;

    // virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) = 0;
};

} // namespace wmtk
