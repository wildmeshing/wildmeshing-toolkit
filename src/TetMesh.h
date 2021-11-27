//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <wmtk/VectorUtils.h>
#include <wmtk/Logger.hpp>

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
		// Cell Tuple Navigator
		class Tuple
		{
		private:
			static constexpr std::array<std::array<int, 2>, 6> m_local_edges = {{{{0, 1}}, {{1, 2}}, {{2, 0}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}}; // local edges within a tet
			static constexpr std::array<int, 6> m_map_edge2face = {{0, 0, 0, 1, 2, 1}};
			static constexpr std::array<std::array<int, 3>, 6> m_local_faces = {{{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 1}}, {{3, 2, 1}}}};

			size_t m_vid;
			size_t m_eid;
			size_t m_fid;
			size_t m_tid;

			std::map<std::vector<int>, int> m_map_local_edges; // DP: what is this?

			int m_timestamp = 0;

		public:
			void update_version_number(const TetMesh &m)
			{
				assert(m_timestamp >= m.m_tet_connectivity[m_tid].timestamp);
				m_timestamp = m.m_tet_connectivity[m_tid].timestamp;
			}

			int get_version_number()
			{
				return m_timestamp;
			}

			bool is_version_number_valid(const TetMesh &m) const
			{
				if (m_timestamp != m.m_tet_connectivity[m_tid].timestamp)
					return false;
				return true;
			}

			void print_info()
			{
				logger().trace("tuple: {} {} {} {}", m_vid, m_eid, m_fid, m_tid);
			}

			// DP: Why do we need this one? if we really need it it should be private.
			static Tuple init_from_edge(const TetMesh &m, int tid, int local_eid)
			{
				int vid = m.m_tet_connectivity[tid][m_local_edges[local_eid][0]];
				int fid = m_map_edge2face[local_eid];
				return Tuple(vid, local_eid, fid, tid);
			}

			Tuple()
			{
			}

			Tuple(size_t vid, size_t eid, size_t fid, size_t tid)
				: m_vid(vid), m_eid(eid), m_fid(fid), m_tid(tid)
			{
			} // DP: the counter should be initialized here?

			inline size_t vid() const { return m_vid; } // update eid and fid
			inline size_t eid() const
			{
				throw "Not implemented";
				return m_eid;
			}
			inline size_t fid() const
			{
				throw "Not implemented";
				return m_fid;
			}
			inline size_t tid() const { return m_tid; }

			// DP: we need to discuss this one, is it used often? why not implement it using switch?
			inline std::vector<Tuple> get_conn_tets(const TetMesh &m) const
			{
				std::vector<Tuple> locs;
				for (int t_id : m.m_vertex_connectivity[m_vid].m_conn_tets)
				{
					locs.push_back(Tuple(m.m_tet_connectivity[t_id][0], 0, 0, t_id));
				}
				return locs;
			}

			inline Tuple switch_vertex(const TetMesh &m) const
			{
				Tuple loc = *this;
				int l_vid1 = m_local_edges[m_eid][0];
				int l_vid2 = m_local_edges[m_eid][1];
				loc.m_vid = m.m_tet_connectivity[m_tid][l_vid1] == m_vid ? m.m_tet_connectivity[m_tid][l_vid2] : m.m_tet_connectivity[m_tid][l_vid1];

				return loc;
			} // along edge

			inline Tuple switch_edge(const TetMesh &m) const
			{
				// TODO
				throw "Not implemented";
			}

			inline Tuple switch_face(const TetMesh &m) const
			{
				// TODO
				throw "Not implemented";
			}

			inline Tuple switch_tetrahedron(const TetMesh &m) const
			{
				// TODO
				throw "Not implemented";
			}
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

			void set_version_number(int version)
			{
				timestamp = version;
			}
			int get_version_number()
			{
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

		void init(size_t n_vertices, const std::vector<std::array<size_t, 4>> &tets);

		bool split_edge(const Tuple &t, std::vector<Tuple> &new_edges);
		void collapse_edge(const Tuple &t);
		void swap_edge(const Tuple &t, int type);

		void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices

		void reset_timestamp()
		{
			m_timestamp = 0;
			for (auto &t : m_tet_connectivity)
				t.timestamp = 0;
		}

		std::vector<Tuple> get_edges() const;

		inline size_t n_tets() const { return m_tet_connectivity.size(); }
		inline size_t v_id(int tid, int lvid) const { return m_tet_connectivity[tid][lvid]; }

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
		virtual bool split_before(const Tuple &t) { return true; } // check edge condition
		// This function computes the attributes for the added simplices
		// if it returns false then the operation is undone
		virtual bool split_after(const std::vector<Tuple> &locs) { return true; } // check tet condition

		//        //// Collapse the edge in the tuple
		//        // Checks if the collapse should be performed or not (user controlled)
		//        virtual bool collapse_before(const Tuple &t) { return true; }
		//        // If it returns false then the operation is undone (the tuple indexes a vertex and tet that survived)
		//        virtual bool collapse_after(const Tuple &t) { return true; }
		//        //todo: quality, inversion, envelope: change v1 pos before this, only need to change partial attributes
		//
		//        //// Swap the edge in the tuple
		//        // Checks if the swapping should be performed or not (user controlled)
		//        virtual bool swapping_before(const Tuple &t) { return true; }
		//        // If it returns false then the operation is undone (the tuple indexes TODO)
		//        virtual bool swapping_after(const Tuple &t) { return true; }
		//        //quality, inversion
		//
		//        // Invariants that are called on all the new or modified elements after an operation is performed
		//        virtual bool VertexInvariant(const Tuple &t) { return true; }
		//        virtual bool EdgeInvariant(const Tuple &t) { return true; }
		//        virtual bool FaceInvariant(const Tuple &t) { return true; }
		//        virtual bool TetrahedronInvariant(const Tuple &t) { return true; }

		virtual void resize_attributes(size_t v, size_t e, size_t f, size_t t) {}

	public:
		inline Tuple switch_vertex(const Tuple &t) const { return t.switch_vertex(*this); }
		inline Tuple switch_edge(const Tuple &t) const { return t.switch_edge(*this); }
		inline Tuple switch_face(const Tuple &t) const { return t.switch_face(*this); }
		inline Tuple switch_tetrahedron(const Tuple &t) const { return t.switch_tetrahedron(*this); }
	};

} // namespace wmtk
