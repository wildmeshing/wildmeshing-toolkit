//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Parameters.h"
#include "Envelope.h"
namespace wmtk {

    class TetMesh {
    public:
        // Cell Tuple Navigator
        class Tuple {
            size_t vid;
            size_t eid;
            size_t fid;
            size_t tid;

        public:
            //todo: where to put these
            std::array<std::array<int, 2>, 6> l_edges = {{{{0, 1}}, {{1, 2}}, {{2, 0}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};
            std::array<std::array<int, 3>, 6> l_faces = {{{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 1}}, {{3, 2, 1}}}};

            std::map<std::vector<int>, int> map_l_edges;

            Tuple(){
                for (int i = 0; i < 6; i++) {
                    map_l_edges[{l_edges[i][0], l_edges[i][1]}] = i;
                    //todo
                }
                //todo map face
            }
            Tuple(size_t _vid, size_t _eid, size_t _fid, size_t _tid): vid(_vid), eid(_eid), fid(_fid), tid(_tid){}

            inline size_t get_vid() const { return vid; }
            inline size_t get_eid() const { return eid; }
            inline size_t get_fid() const { return fid; }
            inline size_t get_tid() const { return tid; }

            inline std::vector<Tuple> get_conn_tets(const TetMesh &m) const {
                std::vector<Tuple> locs;
                for(int t_id: m.m_vertex_connectivity[vid].m_conn_tets){
                    locs.push_back(Tuple(m.m_tet_connectivity[t_id][0], 0, 0, t_id));
                }
                return locs;
            }

            inline Tuple switch_vertex(const TetMesh &m) {
                Tuple loc = *this;
                int l_vid1 = l_edges[eid][0];
                int l_vid2 = l_edges[eid][1];
                loc.vid = m.m_tet_connectivity[tid][l_vid1] == vid ?
                          m.m_tet_connectivity[tid][l_vid2] :
                          m.m_tet_connectivity[tid][l_vid1];

                return loc;
            }//along edge

            Tuple switch_edge(const TetMesh &m);//along face

            Tuple switch_face(const TetMesh &m);//along tet

            Tuple switch_tetrahedron(const TetMesh &m);

            size_t get_vertex_attribute_id(const TetMesh &m);
            size_t get_edge_attribute_id(const TetMesh &m);
            size_t get_face_attribute_id(const TetMesh &m);
            size_t get_tetrahedron_attribute_id(const TetMesh &m);

            inline void set_l_eid(const TetMesh &m, const std::vector<size_t>& vids){
                std::vector<int> l_vids;
                for(int vid0: vids) {
                    for (int j = 0; j < 4; j++)
                        if(vid0 == m.m_tet_connectivity[tid][j])
                            l_vids.push_back(j);
                }
                if(l_vids[0]>l_vids[1])
                    std::swap(l_vids[0], l_vids[1]);

                if(vids.size() == 1){
                    //todo
                } else if (vids.size() == 2) {
                    eid = map_l_edges[l_vids];
                }
            }
            inline void set_l_fid(const TetMesh &m, const std::vector<size_t>& vids){
                std::vector<int> l_vids;
                for(int vid0: vids) {
                    for (int j = 0; j < 4; j++)
                        if(vid0 == m.m_tet_connectivity[tid][j])
                            l_vids.push_back(j);
                }

                if(vids.size() == 1){
                    //todo
                } else if (vids.size() == 2){
                    //todo
                } else if (vids.size() == 3){
                    //todo
                }
            }
        };

        class VertexConnectivity {
        public:
            std::vector<size_t> m_conn_tets;
            bool m_is_removed = false;

            inline size_t &operator[](const size_t index) {
                assert(index >= 0 && index < m_conn_tets.size());
                return m_conn_tets[index];
            }

            inline size_t operator[](const size_t index) const {
                assert(index >= 0 && index < m_conn_tets.size());
                return m_conn_tets[index];
            }
        };

        class TetrahedronConnectivity {
        public:
            std::array<size_t, 4> m_indices;
            bool m_is_removed = false;

            inline size_t &operator[](size_t index) {
                assert(index >= 0 && index < 4);
                return m_indices[index];
            }

            inline size_t operator[](size_t index) const {
                assert(index >= 0 && index < 4);
                return m_indices[index];
            }

            inline int find(int v_id) const {
                for(int j=0;j<4;j++) {
                    if (v_id == m_indices[j])
                        return j;
                }
                return -1;
            }
        };

        class InfoCache{
        public:
            virtual ~InfoCache(){}
        };

        TetMesh(){}
        virtual ~TetMesh() {}

        inline void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets) {
            m_vertex_connectivity.resize(n_vertices);
            m_tet_connectivity.resize(tets.size());
            for (int i = 0; i < tets.size(); i++) {
                m_tet_connectivity[i].m_indices = tets[i];
                for (int j = 0; j < 4; j++)
                    m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
            }
        }

        void split_all_edges();
        bool split_edge(const Tuple &t);

        void collapse_edge(const Tuple &t);
        void swapping_edge(const Tuple &t, int type);

    protected:
        // Stores the connectivity of the mesh
        std::vector<VertexConnectivity> m_vertex_connectivity;
        std::vector<TetrahedronConnectivity> m_tet_connectivity;

        int m_t_empty_slot = 0;
        int m_v_empty_slot = 0;
        int find_next_empty_slot_t();
        int find_next_empty_slot_v();

        //// Split the edge in the tuple
        // Checks if the split should be performed or not (user controlled)
        virtual bool split_before(const Tuple &t, std::shared_ptr<InfoCache> info) = 0;//check edge condition
        // This function computes the attributes for the added simplices
        // if it returns false then the operation is undone
        virtual bool split_after(const Tuple &t, std::shared_ptr<InfoCache> info) = 0;//check tet condition

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

        virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) = 0;

        void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices
    };

}
