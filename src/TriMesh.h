//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>

#include <array>
#include <vector>
#include <map>
#include <cassert>

namespace wmtk
{

	class TriMesh
	{
	public:
		// Cell Tuple Navigator
		class Tuple
		{
		private:
			size_t vid;
			size_t eid;
			size_t fid;
			size_t m_counter;

		public:
            void print_info(){
                cout<<vid<<" "<<eid<<" "<<fid<<endl;
            }
    
//        v2
//      /    \ 
// e1  /      \  e0
//    v0 - - - v1
//        e2

			Tuple()
			{
			}
			Tuple(size_t _vid, size_t _eid, size_t _fid) : vid(_vid), eid(_eid), fid(_fid) {}

			inline size_t get_vid() const { return vid; }
			inline size_t get_eid() const { return eid; }
			inline size_t get_fid() const { return fid; }

			inline Tuple switch_vertex(const TriMesh &m)
			{
				assert(is_valid());

				const int v0 = m_tri_connectivity[fid][0];
				const int v1 = m_tri_connectivity[fid][1];
				const int v2 = m_tri_connectivity[fid][2];

				Tuple loc = *this;
				switch(eid)
				{
					case 0:
						assert(vid == v1 || vid == v2);
						loc.vid = vid == v1 ? v2 : v1;
						break;
					case 1:
						assert(vid == v0 || vid == v2);
						loc.vid = vid == v0 ? v2 : v0;
						break;
					case 2:
						assert(vid == v0 || vid == v1);
						loc.vid = vid == v0 ? v1 : v0;
						break;
				}

				assert(loc.is_valid());
				return loc;
			}

			Tuple switch_edge(const TriMesh &m)
			{
				assert(is_valid());

				const int lvid = m_tri_connectivity[fid].find(vid);
				assert(lvid == 0 || lvid == 1 || lvid == 2);

				Tuple loc = *this;
				switch(lvid)
				{
					case 0:
						assert(eid == 1 || eid == 2);
						loc.eid = eid == 1 ? 2 : 1;
						break;
					case 1:
						assert(eid == 0 || eid == 2);
						loc.eid = eid == 0 ? 2 : 0;
						break;
					case 2:
						assert(eid == 0 || eid == 1);
						loc.eid = eid == 0 ? 1 : 0;
						break;
				}

				assert(loc.is_valid());
				return loc;
			}



			Tuple switch_face(const TriMesh &m)
			{
				assert(is_valid());

				const v0 = vid;
				const v1 = this->switch_vertex(m).vid;

				// Intersect the 1-ring of the two vertices in the edge pointed by the tuple
				std::vector<size_t>& v0_tids = m.m_vertex_connectivity[v0].m_conn_tris;
				std::vector<size_t>& v1_tids = m.m_vertex_connectivity[v1].m_conn_tris;  

				std::sort(v0_tids.begin(), v0_tids.end());
    			std::sort(v1_tids.begin(), v1_tids.end());
 			    std::vector<int> tids;
     			std::set_intersection(v0_tids.begin(), v0_tids.end(), v1_tids.begin(), v1_tids.end(), std::back_inserter(tids)); // make sure this is correct
				assert(tids.size() == 1 || tids.size() == 2);

				if (tids.size() == 1)
					return *this;

				Tuple loc = *this;

				// There is a triangle on the other side
				if (tids.size() == 2)
				{
					// Find the fid of the triangle on the other side
					size_t tid2 = tids[0] == tid ? tids[1] : tids[0];
					loc.fid = tid2

					// Get sorted local indices of the two vertices in the new triangle
					size_t lv0_2 = m_tri_connectivity[fid2].find(v0);
					assert (lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
					size_t lv1_2 = m_tri_connectivity[fid2].find(v1);
					assert (lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);
					
					if (lv0_2 > lv1_2)
						std::swap(lv0_2,lv1_2);

					// Assign the edge id depending on the table
					if (lv0_2 == 0 && lv1_2 == 1)
					{
						loc.eid = 2;
					}
					else if (lv0_2 == 1 && lv1_2 == 2)
					{
						loc.eid = 0;
					}
					else if (lv0_2 == 0 && lv1_2 == 2)
					{
						loc.eid = 1;
					} else {assert(false);}
	 
					loc.m_counter = m.m_tri_connectivity[loc.fid].m_counter;

				}

				assert(loc.is_valid());
				return loc;
			}

			bool is_valid(const TriMesh &m)
			{
				// Condition 0: Elements exist
				if (vid < 0 || vid >= m.m_vert_connectivity.size())
					return false;

				if (eid < 0 || eid > 2)
					return false;

				if (fid < 0 || fid > m.m_tri_connectivity.size())
					return false;

				// Condition 1: tid and vid are consistent
				const int lvid = m_tri_connectivity[fid].find(vid);
				if (!(lvid == 0 || lvid == 1 || lvid == 2))
					return false;

				// Condition 2: eid is valid
				const int v0 = m_tri_connectivity[fid][0];
				const int v1 = m_tri_connectivity[fid][1];
				const int v2 = m_tri_connectivity[fid][2];

				switch(eid)
				{
					case 0:
						if(!(vid == v1 || vid == v2))
							return false;
						break;
					case 1:
						if(!(vid == v0 || vid == v2))
							return false;
						break;
					case 2:
						if(!(vid == v0 || vid == v1))
							return false;
						break;
				}

				// Condition 3: the counter is up to date
				if (m_counter != m.mm_tri_connectivity[tid].m_counter)
					return false;

				return true;
			}

			size_t get_vertex_attribute_id(const TriMesh &m);
			size_t get_edge_attribute_id(const TriMesh &m);
			size_t get_face_attribute_id(const TriMesh &m);
		};

		class VertexConnectivity
		{
		public:
			std::vector<size_t> m_conn_tris;
			bool m_is_removed = false;

			inline size_t &operator[](const size_t index)
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
			size_t m_counter;

			inline size_t &operator[](size_t index)
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
				for (int j = 0; j < 3; j++)
				{
					if (v_id == m_indices[j])
						return j;
				}
				return -1;
			}
		};

		TriMesh() {}
		virtual ~TriMesh() {}

		inline void create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 4>> &tris)
		{
			m_vertex_connectivity.resize(n_vertices);
			m_tri_connectivity.resize(tris.size());
			for (int i = 0; i < tris.size(); i++)
			{
				m_tri_connectivity[i].m_indices = tris[i];
				for (int j = 0; j < 3; j++)
					m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
			}
		}

		// REMOVE ME!!!!
		void split_all_edges();

		bool split_edge(const Tuple &t);
		void collapse_edge(const Tuple &t);
		void swap_edge(const Tuple &t, int type);

		void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices

		vector<Tuple> get_all_vertices();
		vector<Tuple> get_all_edges();
		vector<Tuple> get_all_tris();


	protected: // THESE SHOULD BE PRIVATE!!!!
		// Stores the connectivity of the mesh
		std::vector<VertexConnectivity> m_vertex_connectivity;
		std::vector<TriangleConnectivity> m_tri_connectivity;

		int m_t_empty_slot = 0;
		int m_v_empty_slot = 0;
		int find_next_empty_slot_t();
		int find_next_empty_slot_v();

	protected:
		//// Split the edge in the tuple
		// Checks if the split should be performed or not (user controlled)
		virtual bool split_before(const Tuple &t) = 0; // check edge condition
		// This function computes the attributes for the added simplices
		// if it returns false then the operation is undone
		virtual bool split_after(const Tuple &t) = 0; // check tet condition

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
