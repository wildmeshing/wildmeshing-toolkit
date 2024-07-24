
#include "find_local_dart_action.hpp"

#include <cassert>
#include <optional>
#include <stdexcept>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "local_switch_tuple.hpp"
namespace wmtk::multimesh::utils {
int8_t find_local_dart_action(PrimitiveType pt, const Tuple& source, const Tuple& target)
{
    wmtk::autogen::SimplexDart sd(pt);
    return find_local_dart_action(sd, source, target);
}
int8_t find_local_dart_action(
    const wmtk::autogen::SimplexDart& sd,
    const Tuple& source,
    const Tuple& target)
{
    // target = R * source
    // target * source^{-1} = R
    int8_t src = sd.valid_index_from_tuple(source);
    int8_t tgt = sd.valid_index_from_tuple(target);
    int8_t src_inv = sd.inverse(src);
    return tgt * src_inv;
}

} // namespace wmtk::multimesh::utils
