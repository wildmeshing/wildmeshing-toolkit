#pragma once

#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/ConstAccessor.hpp>

namespace wmtk::function::utils {

template <typename T>

/**
 * @brief get attributes from a simplex
 *
 * @param mesh mesh
 * @param accessor accessor from where to read the attributes
 * @param primitive_type type of the data to extract (eg if vertex it will extract the vertex
 * attributes)
 * @param simplex source simples (eg is simplex is triangle and primitive_type is vertex it extracts
 * the vertex attributes for that triangle)
 * @param vertex_marker optional, a tuple pointing to one of the simplicies of type primitive_type
 * in the simplex (eg one of the vertices of the triangle)
 *
 * @return a pair containing a vector of Vectors one for each attribute (eg 3 for a triangle) and an
 * index poiting to the attr corresponding to the vertex_marker (-1 if not specified)
 */
std::tuple<std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>>, long> get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<T>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker = {});
}