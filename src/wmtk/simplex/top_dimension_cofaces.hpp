#pragma once

#include "SimplexCollection.hpp"

namespace wmtk::simplex {

void top_dimension_cofaces(
    const Simplex& simplex,
    SimplexCollection& simplex_collection,
    const bool sort_and_clean = true);


void top_dimension_cofaces_tuples(
    const EdgeMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);

void top_dimension_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);

void top_dimension_cofaces_tuples(
    const TetMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);

void top_dimension_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);


void top_dimension_cofaces_tuples(
    const EdgeMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection);

void top_dimension_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection);

void top_dimension_cofaces_tuples(
    const TetMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection);

void top_dimension_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection);

// Returns one simplex for every top level coface of the input simplex.
// The tuple held by each Simplex is guaranteed to contain the child simplex (and all oother lower
// order simplices of the tuple) In particular, for any simplex (t, k) for tuple t, dimension k,
// this function returns simplices (u,m) such that \forall i \leq k, id(t,i) = id(u,i)
SimplexCollection top_dimension_cofaces(
    const TriMesh& mesh,
    const Simplex& simplex,
    const bool sort_and_clean = true);

SimplexCollection top_dimension_cofaces(
    const TetMesh& mesh,
    const Simplex& simplex,
    const bool sort_and_clean = true);

SimplexCollection
top_dimension_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

std::vector<Tuple> top_dimension_cofaces_tuples(const EdgeMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const Mesh& mesh, const Simplex& simplex);


} // namespace wmtk::simplex
