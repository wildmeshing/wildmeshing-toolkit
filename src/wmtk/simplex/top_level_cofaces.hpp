#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {


// Returns one simplex for every top level coface of the input simplex.
// The tuple held by each Simplex is guaranteed to contain the child simplex (and all oother lower
// order simplices of the tuple) In particular, for any simplex (t, k) for tuple t, dimension k,
// this function returns simplices (u,m) such that \forall i \leq k, id(t,i) = id(u,i)
SimplexCollection
top_level_cofaces(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
top_level_cofaces(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection
top_level_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);


// variants of top_level_cofaces that just use a std::vector<Tuple> because every return value has
// the same simplex dimension
std::vector<Tuple> top_level_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_level_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_level_cofaces_tuples(const Mesh& mesh, const Simplex& simplex);

} // namespace wmtk::simplex
