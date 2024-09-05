
#include "find_local_dart_action.hpp"

#include <cassert>
#include <optional>
#include <stdexcept>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
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
    return wmtk::autogen::find_local_dart_action(sd, src, tgt);
}
int8_t find_local_dart_action(
    const wmtk::autogen::SimplexDart& sd,
    const wmtk::autogen::Dart& source,
    const wmtk::autogen::Dart& target)
{
    return wmtk::autogen::find_local_dart_action(
        sd,
        source.local_orientation(),
        target.local_orientation());
}

} // namespace wmtk::multimesh::utils
