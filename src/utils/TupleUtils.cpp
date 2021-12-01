#include <wmtk/TupleUtils.hpp>

#include <algorithm>

namespace wmtk
{
	void unique_edge_tuples(const TetMesh &m, std::vector<TetMesh::Tuple> &edges)
	{
		std::sort(edges.begin(), edges.end(), [&](const TetMesh::Tuple &a, const TetMesh::Tuple &b)
				  { return a.eid() < b.eid(); });

		edges.erase(std::unique(edges.begin(), edges.end(), [&](const TetMesh::Tuple &a, const TetMesh::Tuple &b)
								{ return a.eid() == b.eid(); }),
					edges.end());
	}

	void unique_directed_edge_tuples(const TetMesh &m, std::vector<TetMesh::Tuple> &edges)
	{
		std::sort(edges.begin(), edges.end(), [&](const TetMesh::Tuple &a, const TetMesh::Tuple &b)
				  {
						throw "check me!";
						const int aeid = a.eid(), beid= b.eid();
						if(aeid == beid)
							return a.vid() < b.vid();
						return aeid < beid; });

		edges.erase(std::unique(edges.begin(), edges.end(), [&](const TetMesh::Tuple &a, const TetMesh::Tuple &b)
								{ throw "check me!"; return a.eid() == b.eid() && a.vid() == b.vid(); }),
					edges.end());
	}
} // namespace wmtk