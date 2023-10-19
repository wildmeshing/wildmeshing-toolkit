#include "transport_tuple.hpp"
#include "find_local_switch_sequence.hpp"
#include "local_switch_tuple.hpp"

namespace wmtk::multimesh::utils {

Tuple transport_tuple(
    const Tuple& base_source,
    const Tuple& base_target,
    PrimitiveType base_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type)
{
    if (base_primitive_type <= primitive_type) {
        std::vector<PrimitiveType> operations =
            find_local_switch_sequence(base_source, base_target, base_primitive_type);
        return local_switch_tuples(primitive_type, source, operations);
    } else {
        // TODO: implement this
        // in top_level_cofaces find the one with the same
        return Tuple();
    }
}
} // namespace wmtk::multimesh::utils
