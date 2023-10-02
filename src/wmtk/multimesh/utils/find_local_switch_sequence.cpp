#include "find_local_switch_sequence.hpp"
namespace wmtk::multimesh::utils {


namespace {

std::vector<PrimitiveType> find_local_switch_sequence_trimesh(
    const Tuple& source,
    const Tuple& target)
{
    // TODO: assert that base_source and base_target use the same global CID

    // circulate
    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;
    auto try_and_record = [&](PrimitiveType pt) -> bool {
        cur_tuple = local_switch_tuple(cur_tuple, pt);
        switches.emplace_back(pt);
        return cur_tuple == target;
    };
    for (long j = 0; j < 3; ++j) {
        for (PrimitiveType pt : {PrimtiiveType::Vertex, PrimitiveType::Edge}) {
            if (try_and_record(pt)) {
                return switches;
            }
        }
    }
    throw "switch sequence was unable to find a sequence of switches to match tuples";
    return switches;
}
std::vector<PrimitiveType> find_local_switch_sequence_edgemesh(
    const Tuple& source,
    const Tuple& target)
{
    if(source != target) {
        return std::vector<PrimitiveType>{PrimitiveType::switch_vertex};
    } else {
        return std::vector<PrimitiveType>{};
    }
    throw "switch sequence was unable to find a sequence of switches to match tuples";
}
} // namespace

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
std::vector<PrimitiveType>
find_local_switch_sequence(const Tuple& source, const Tuple& target, PrimitiveType primitive_type)
{
    switch (primitive_type) {
    case PrimitiveType::Face: return find_local_switch_sequence_trimesh(source, target);
    case PrimitiveType::Edge: return find_local_switch_sequence_edgemesh(source, target);
    default: return {};
    }
}
} // namespace wmtk::multimesh::utils
