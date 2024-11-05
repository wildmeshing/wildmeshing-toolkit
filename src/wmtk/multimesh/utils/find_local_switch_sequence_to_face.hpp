#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {


// finds a sequence to find an equivalent subtuple that utilizes the given face
std::vector<PrimitiveType> find_local_switch_sequence_to_face(
    const Tuple& source,
    const Tuple& target,
    PrimitiveType subtuple_dimension,
    PrimitiveType primitive_type);

namespace internal {
// interface for using this function with local boundary indices
std::vector<PrimitiveType> find_local_switch_sequence_to_face(
    const Tuple& source,
    int8_t local_boundary_index,
    PrimitiveType subtuple_dimension,
    PrimitiveType primitive_type);
} // namespace internal
} // namespace wmtk::multimesh::utils
