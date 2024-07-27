#include "transport_tuple.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include "find_local_dart_action.hpp"
#include "find_local_switch_sequence.hpp"
#include "local_switch_tuple.hpp"

namespace wmtk::multimesh::utils {

namespace internal {
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
    return transport_tuple(base_sd, sd, base_source, base_target, source);
}
} // namespace internal
Tuple transport_tuple(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type)
{
    return internal::transport_tuple_sequence(
        base_source,
        base_target,
        base_primitive_type,
        source,
        primitive_type);
}
Tuple transport_tuple(
    const wmtk::autogen::SimplexDart& base_sd,
    const wmtk::autogen::SimplexDart& sd,
    const Tuple& base_source,
    const Tuple& base_target,
    const Tuple& source)
{
    const int8_t base_action = find_local_dart_action(base_sd, base_source, base_target);
    const int8_t action = base_sd.convert(base_action, sd);


    int8_t src_dart = sd.valid_index_from_tuple(source);
    const int8_t tgt_dart = sd.product(action, src_dart);
    return sd.update_tuple_from_valid_index(source, tgt_dart);
}
} // namespace wmtk::multimesh::utils
