#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
Tuple transport_tuple(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type);
} // namespace wmtk::multimesh::utils
