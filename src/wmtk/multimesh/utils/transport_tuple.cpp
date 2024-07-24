#include "transport_tuple.hpp"
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include "find_local_switch_sequence.hpp"
#include "local_switch_tuple.hpp"

namespace wmtk::multimesh::utils {

Tuple transport_tuple_sequence(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type)
{
    std::vector<PrimitiveType> operations =
        find_local_switch_sequence(base_source, base_target, base_primitive_type);
    return local_switch_tuples(primitive_type, source, operations);
}
Tuple transport_tuple_dart(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type)
{
    wmtk::autogen::SimplexDart base_sd(base_primitive_type);
    wmtk::autogen::SimplexDart sd(primitive_type);

    int8_t action = find_local_dart_action(base_sd, base_source, base_target);
}
} // namespace wmtk::multimesh::utils
