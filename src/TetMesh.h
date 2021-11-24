//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>

#include <array>
#include <vector>
#include <map>
#include <cassert>
#include <queue>

namespace wmtk
{
	class TetMesh
	{
	public:
        const std::array<std::array<int, 2>, 6> local_edges = {{{{0, 1}}, {{1, 2}}, {{2, 0}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};//local edges within a tet
        const std::array<int, 6> map_edge2face = {{0, 0, 0, 1, 2, 1}};
        const std::array<std::array<int, 3>, 6> local_faces = {{{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 1}}, {{3, 2, 1}}}};

		// Cell Tuple Navigator
		class Tuple
		{
		private:
			size_t vid;
			size_t eid;
			size_t fid;
			size_t tid;

			std::map<std::vector<int>, int> map_local_edges; // DP: what is this?

            int timestamp = 0;

        public:

            void update_version_number(const TetMesh &m){
                timestamp = m.m_tet_connectivity[tid].timestamp;
            }
            int get_version_number(){
                return timestamp;
            }
            bool is_version_number_valid(const TetMesh &m) const {
                if(timestamp != m.m_tet_connectivity[tid].timestamp)
                    return false;
                return true;
            }

			// DP: we need a is_valid function similar to the 2D version

            void print_info(){
                cout<<vid<<" "<<eid<<" "<<fid<<" "<<tid<<endl; // DP: no cout
            }

			// DP: Why do we need this one? if we really need it it should be private.
            static Tuple get_edge_tuple(const TetMesh &m, int _tid, int local_eid){
                int _vid = m.m_tet_connectivity[_tid][m.local_edges[local_eid][0]];
                int _fid = m.map_edge2face[local_eid];
                return Tuple(_vid, local_eid, _fid, _tid);
            }

			// DP: Why do we need this one? if we really need it it should be private.
            static int compare_edges(const TetMesh &m, const Tuple& loc1, const Tuple& loc2) {
                std::array<size_t, 2> e1 = {{m.m_tet_connectivity[loc1.tid][m.local_edges[loc1.eid][0]],
                                          m.m_tet_connectivity[loc1.tid][m.local_edges[loc1.eid][1]]}};
                std::array<size_t, 2> e2 = {{m.m_tet_connectivity[loc2.tid][m.local_edges[loc2.eid][0]],
                                          m.m_tet_connectivity[loc2.tid][m.local_edges[loc2.eid][1]]}};
                if (e1[0] > e1[1])
                    std::swap(e1[0], e1[1]);
                if (e2[0] > e2[1])
                    std::swap(e2[0], e2[1]);
                if (e1 < e2)
                    return -1;
                else if (e1 == e2)
                    return 0;
                else
                    return 1;
            }

			// DP: Why do we need this one? if we really need it it should be private.
            static int compare_directed_edges(const TetMesh &m, const Tuple& loc1, const Tuple& loc2) {
                std::array<size_t, 2> e1 = {{m.m_tet_connectivity[loc1.tid][m.local_edges[loc1.eid][0]],
                                             m.m_tet_connectivity[loc1.tid][m.local_edges[loc1.eid][1]]}};
                std::array<size_t, 2> e2 = {{m.m_tet_connectivity[loc2.tid][m.local_edges[loc2.eid][0]],
                                             m.m_tet_connectivity[loc2.tid][m.local_edges[loc2.eid][1]]}};
                if (e1 < e2)
                    return -1;
                else if (e1 == e2)
                    return 0;
                else
                    return 1;
            }

			// DP: Why do we need this one? if we really need it it should be private.
            static void unique_edge_tuples(const TetMesh &m, std::vector<Tuple>& edges){
                std::sort(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
                    return compare_edges(m, a, b) < 0;
                });
                edges.erase(std::unique(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
                    return compare_edges(m, a, b) == 0;
                }), edges.end());
            }

			// DP: Why do we need this one? if we really need it it should be private.
            static void unique_directed_edge_tuples(const TetMesh &m, std::vector<Tuple>& edges){
                std::sort(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
                    return compare_directed_edges(m, a, b) < 0;
                });
                edges.erase(std::unique(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
                    return compare_directed_edges(m, a, b) == 0;
                }), edges.end());
            }

			Tuple(){}
			Tuple(size_t _vid, size_t _eid, size_t _fid, size_t _tid) : vid(_vid), eid(_eid), fid(_fid), tid(_tid) {} // DP: the counter should be initialized here?

			inline size_t get_vid() const { return vid; } // DP: these should not be exposed
			inline size_t get_eid() const { return eid; }
			inline size_t get_fid() const { return fid; }
			inline size_t get_tid() const { return tid; }

			// DP: we need to discuss this one, is it used often? why not implement it using switch?
			inline std::vector<Tuple> get_conn_tets(const TetMesh &m) const
			{
				std::vector<Tuple> locs;
				for (int t_id : m.m_vertex_connectivity[vid].m_conn_tets)
				{
					locs.push_back(Tuple(m.m_tet_connectivity[t_id][0], 0, 0, t_id));
				}
				return locs;
			}

			inline Tuple switch_vertex(const TetMesh &m)
			{
				Tuple loc = *this;
				int l_vid1 = m.local_edges[eid][0];
				int l_vid2 = m.local_edges[eid][1];
				loc.vid = m.m_tet_connectivity[tid][l_vid1] == vid ? m.m_tet_connectivity[tid][l_vid2] : m.m_tet_connectivity[tid][l_vid1];

				return loc;
			} // along edge

			// DP: this is missing
			Tuple switch_edge(const TetMesh &m); // along face

			// DP: this is missing
			Tuple switch_face(const TetMesh &m); // along tet

			// DP: this is missing
			Tuple switch_tetrahedron(const TetMesh &m);

			// DP: how do you access properties without these?
			size_t get_vertex_attribute_id(const TetMesh &m);
			size_t get_edge_attribute_id(const TetMesh &m);
			size_t get_face_attribute_id(const TetMesh &m);
			size_t get_tetrahedron_attribute_id(const TetMesh &m);
		};

		class VertexConnectivity
		{
		public:
			std::vector<size_t> m_conn_tets;
			bool m_is_removed = false;

			inline size_t &operator[](const size_t index)
			{
				assert(index >= 0 && index < m_conn_tets.size());
				return m_conn_tets[index];
			}

			inline size_t operator[](const size_t index) const
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

            void set_version_number(int version){
                timestamp = version;
            }
            int get_version_number(){
                return timestamp;
            }

			inline size_t &operator[](size_t index)
			{
				assert(index >= 0 && index < 4);
				return m_indices[index];
			}

			inline size_t operator[](size_t index) const
			{
				assert(index >= 0 && index < 4);
				return m_indices[index];
			}

			inline int find(int v_id) const
			{
				for (int j = 0; j < 4; j++)
				{
					if (v_id == m_indices[j])
						return j;
				}
				return -1;
			}
		};

		TetMesh() {}
		virtual ~TetMesh() {}

		inline void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 4>> &tets)
		{
			m_vertex_connectivity.resize(n_vertices);
			m_tet_connectivity.resize(tets.size());
			for (int i = 0; i < tets.size(); i++)
			{
				m_tet_connectivity[i].m_indices = tets[i];
				for (int j = 0; j < 4; j++)
					m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
			}
		}

		bool split_edge(const Tuple &t, std::vector<Tuple>& new_edges);
		void collapse_edge(const Tuple &t);
		void swap_edge(const Tuple &t, int type);

		void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices

	protected: // THESE SHOULD BE PRIVATE!!!!
		// Stores the connectivity of the mesh
		std::vector<VertexConnectivity> m_vertex_connectivity;
		std::vector<TetrahedronConnectivity> m_tet_connectivity;

		int m_t_empty_slot = 0;
		int m_v_empty_slot = 0;
		int find_next_empty_slot_t();
		int find_next_empty_slot_v();

        int timestamp = 0;
        void reset_timestamp(){
            timestamp = 0;
            for(auto& t: m_tet_connectivity)
                t.timestamp = 0;
        }

	protected:
		//// Split the edge in the tuple
		// Checks if the split should be performed or not (user controlled)
		virtual bool split_before(const Tuple &t) = 0; // check edge condition
		// This function computes the attributes for the added simplices
		// if it returns false then the operation is undone
		virtual bool split_after(const std::vector<Tuple> &locs) = 0; // check tet condition

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
	};

} // namespace wmtk
