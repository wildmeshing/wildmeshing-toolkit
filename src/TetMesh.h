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

        Tuple() {}

        Tuple(size_t vid, size_t eid, size_t fid, size_t tid)
            : m_vid(vid)
            , m_eid(eid)
            , m_fid(fid)
            , m_tid(tid)
        {} // DP: the counter should be initialized here?

        static Tuple init_from_edge(const TetMesh& m, int tid, int local_eid);
        static Tuple init_from_vertex(const TetMesh& m, int vid);
        static Tuple init_from_tet(const TetMesh& m, int tid);

        bool is_valid(const TetMesh& m) const;

        void update_version_number(const TetMesh& m);
        int get_version_number();
        bool is_version_number_valid(const TetMesh& m) const;

        void print_info();

        size_t vid() const;

        size_t eid(const TetMesh& m) const;
        size_t fid(const TetMesh& m) const;
        size_t tid() const;

        Tuple switch_vertex(const TetMesh& m) const;
        Tuple switch_edge(const TetMesh& m) const;
        Tuple switch_face(const TetMesh& m) const;
        std::optional<Tuple> switch_tetrahedron(const TetMesh& m) const;

        std::vector<Tuple> get_conn_tets(const TetMesh& m) const;
        std::array<Tuple, 4> oriented_tet_vertices(const TetMesh& m) const;
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
//    size_t v_id(int tid, int lvid) const { return m_tet_connectivity[tid][lvid]; }

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
    Tuple tuple_from_vertex(int vid) const
    {
        return Tuple::init_from_vertex(*this, vid);
    }
    Tuple tuple_from_tet(int tid) const
    {
        return Tuple::init_from_tet(*this, tid);
    }

    std::array<Tuple, 4> oriented_tet_vertices(const Tuple& t) const
    {
        return t.oriented_tet_vertices(*this);
    }
};

} // namespace wmtk
