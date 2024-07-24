#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {
namespace internal {

Tuple transport_tuple_sequence(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type);
Tuple transport_tuple_dart(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type);
} // namespace internal

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
// base_primitive_type specifies the type of mesh the base_source -> base_target should happen in
// primtmivie_type specifies for what sort of mesh source should be mapped to the return
Tuple transport_tuple(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type);
} // namespace wmtk::multimesh::utils
