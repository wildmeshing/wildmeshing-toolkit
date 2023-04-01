
#pragma once


namespace wmtk {
    class TriMesh;
    // Cell TriMeshTuple Navigator
    class TriMeshTuple
    {
    private:
        friend class TriMeshTriMeshTupleData;
        size_t m_vid = -1;
        size_t m_eid = -1;
        size_t m_fid = -1;
        size_t m_hash = -1;

        void update_hash(const TriMesh& m);

    public:
        void print_info() const;
        std::string info() const;

        //         v2        /
        //       /    \      /
        //  e1  /      \  e0 /
        //     v0 - - - v1   /
        //         e2        /
        /**
         * Construct a new TriMeshTuple object with global vertex/triangle index and local edge index
         *
         * @param vid vertex id
         * @param eid edge id (local)
         * @param fid face id
         * @note edge ordering
         */
        TriMeshTuple() = default;
        TriMeshTuple(const TriMeshTuple& other) = default;
        TriMeshTuple(TriMeshTuple&& other) = default;
        TriMeshTuple& operator=(const TriMeshTuple& other) = default;
        TriMeshTuple& operator=(TriMeshTuple&& other) = default;
        TriMeshTuple(size_t vid, size_t eid, size_t fid, const TriMesh& m)
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
        size_t eid(const TriMesh& m) const;
        size_t eid_unsafe(const TriMesh& m) const;
        /**
         * returns the local eid of the tuple
         *
         * @param m TriMesh where the tuple belongs.
         * @return size_t
         * @note use mostly for constructing consistent tuples in operations
         */
        size_t local_eid(const TriMesh&) const { return m_eid; }
        /**
         * Switch operation.
         *
         * @param m Mesh
         * @return another TriMeshTuple that share the same face, edge, but different vertex.
         */
        TriMeshTuple switch_vertex(const TriMesh& m) const;
        /**
         *
         * @param m
         * @return another TriMeshTuple that share the same face, vertex, but different edge.
         */
        TriMeshTuple switch_edge(const TriMesh& m) const;
        /**
         * Switch operation for the adjacent triangle
         *
         * @param m Mesh
         * @return TriMeshTuple for the edge-adjacent triangle, sharing same edge, and vertex.
         * @note nullopt if the TriMeshTuple of the switch goes off the boundary.
         */
        std::optional<TriMeshTuple> switch_face(const TriMesh& m) const;

        /**
         * @brief check if a TriMeshTuple is valid
         *
         * @param m the Mesh
         * @return false if 1. the fid of the TriMeshTuple is -1, 2. either the vertex or the face
         * refered to by the TriMeshTuple is removed, 3. the hash of the TriMeshTuple is not the same as
         * the hash of the triangle it refers to in the mesh
         *
         */
        bool is_valid(const TriMesh& m) const;

        bool is_ccw(const TriMesh& m) const;
        /**
         * Positively oriented 3 vertices (represented by TriMeshTuples) in a tri.
         * @return std::array<TriMeshTuple, 3> each tuple owns a different vertex.
         */
        std::array<TriMeshTuple, 3> oriented_tri_vertices(const TriMesh& m) const;

        std::tuple<size_t, size_t, size_t, size_t> as_stl_tuple() const
        {
            return std::tie(m_vid, m_eid, m_fid, m_hash);
        }
        friend bool operator<(const TriMeshTuple& a, const TriMeshTuple& t)
        {
            return (
                std::tie(a.m_vid, a.m_eid, a.m_fid, a.m_hash) <
                std::tie(t.m_vid, t.m_eid, t.m_fid, t.m_hash));
            // return a.as_stl_tuple() < t.as_stl_tuple();
        }
    };
}
