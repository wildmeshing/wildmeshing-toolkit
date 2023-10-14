#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
std::vector<PrimitiveType>
find_local_switch_sequence(const Tuple& source, const Tuple& target, PrimitiveType primitive_type);
} // namespace wmtk::multimesh::utils
