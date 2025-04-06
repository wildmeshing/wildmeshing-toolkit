#pragma once

#include "SimplexCollection.hpp"

namespace wmtk {
    class PointMesh;
    class EdgeMesh;
    class TriMesh;
    class TetMesh;
}
namespace wmtk::simplex {

/**
 * @brief Get all top dimension cofaces of the given simplex.
 *
 * For example, the top dimension cofaces in a TetMesh are all tetrahedra incident to the given
 * simplex.
 *
 * The tuple held by each Simplex is guaranteed to contain the child simplex (and all other lower
 * order simplices of the tuple) In particular, for any simplex (t, k) for tuple t, dimension k,
 * this function returns simplices (u,m) such that \forall i \leq k, id(t,i) = id(u,i).
 *
 * @param simplex The simplex of which the cofaces are computed.
 * @param simplex_collection The top dimension cofaces are added to this SimplexCollection.
 *
 */
void top_dimension_cofaces(
    const Simplex& simplex,
    SimplexCollection& simplex_collection,
    const bool sort_and_clean = true);


/**
 * @brief The same as top_dimension_cofaces but it appends to a vector of tuples.
 *
 * As all simplices returned by this function are of the same type, it is often more efficient to
 * just return the vector of tuples instead of creating a SimplexCollection.
 */
void top_dimension_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);

void top_dimension_cofaces_tuples(
    const PointMesh& mesh,
    const Simplex& simplex,
    SimplexCollection& collection);

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
    const PointMesh& mesh,
    const Simplex& simplex,
    std::vector<Tuple>& collection);

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

/**
 * @brief Get all top dimension cofaces of the given simplex.
 *
 * For example, the top dimension cofaces in a TetMesh are all tetrahedra incident to the given
 * simplex.
 *
 * The tuple held by each Simplex is guaranteed to contain the child simplex (and all other lower
 * order simplices of the tuple) In particular, for any simplex (t, k) for tuple t, dimension k,
 * this function returns simplices (u,m) such that \forall i \leq k, id(t,i) = id(u,i).
 *
 * @param mesh The mesh to which the simplex and its cofaces belong.
 * @param simplex The simplex of which the cofaces are computed.
 * @return A SimplexCollection with all top dimension cofaces.
 *
 */
SimplexCollection
top_dimension_cofaces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean = true);

SimplexCollection top_dimension_cofaces(
    const TriMesh& mesh,
    const Simplex& simplex,
    const bool sort_and_clean = true);

SimplexCollection top_dimension_cofaces(
    const TetMesh& mesh,
    const Simplex& simplex,
    const bool sort_and_clean = true);

/**
 * @brief The same as top_dimension_cofaces but it returns only a vector of tuples.
 *
 * As all simplices returned by this function are of the same type, it is often more efficient to
 * just return the vector of tuples instead of creating a SimplexCollection.
 */
std::vector<Tuple> top_dimension_cofaces_tuples(const Mesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const PointMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const EdgeMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const TriMesh& mesh, const Simplex& simplex);

std::vector<Tuple> top_dimension_cofaces_tuples(const TetMesh& mesh, const Simplex& simplex);


} // namespace wmtk::simplex
