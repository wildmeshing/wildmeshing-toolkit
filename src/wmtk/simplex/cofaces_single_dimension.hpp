#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
namespace wmtk {
    class PointMesh;
    class EdgeMesh;
    class TriMesh;
    class TetMesh;
}
namespace wmtk::simplex {

// Returns the cofaces of a provided simplex, but only providing the cofaces in the provided coface
// type

/**
 * @brief Returns all cofaces of a simplex that are of the provided primitive type.
 *
 * The coface tuples are guaranteed to contain the provided simplex in their tuple.
 *
 * @param mesh The mesh containing the simplex.
 * @param my_simplex The simplex whose cofaces we are computing.
 * @param cofaces_type The primitive type of the cofaces.
 *
 * @return A SimplexCollection that contains the cofaces.
 */
SimplexCollection cofaces_single_dimension(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean = true);

std::vector<Tuple> cofaces_single_dimension_tuples(
    const Mesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type);

std::vector<Simplex> cofaces_single_dimension_simplices(
    const Mesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type);

SimplexCollection cofaces_single_dimension(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type,
    bool sort_and_clean = true);

std::vector<Tuple> cofaces_single_dimension_tuples(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    PrimitiveType cofaces_type);

std::vector<Simplex> cofaces_single_dimension_simplices(
    const TriMesh& mesh,
    const Simplex& simplex,
    PrimitiveType cofaces_type);


} // namespace wmtk::simplex
