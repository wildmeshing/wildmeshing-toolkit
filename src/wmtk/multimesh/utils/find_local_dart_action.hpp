#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk::autogen {
class SimplexDart;
}

namespace wmtk::multimesh::utils {

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
int8_t find_local_dart_action(PrimitiveType mesh_type, const Tuple& source, const Tuple& target);
int8_t find_local_dart_action(
    const wmtk::autogen::SimplexDart& sd,
    const Tuple& source,
    const Tuple& target);
} // namespace wmtk::multimesh::utils
