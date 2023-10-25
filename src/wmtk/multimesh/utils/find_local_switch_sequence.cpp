#include "find_local_switch_sequence.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <wmtk/utils/TupleInspector.hpp>
#include "local_switch_tuple.hpp"
namespace wmtk::multimesh::utils {


namespace {
bool hash_free_tuple_equality(const Tuple& a, const Tuple& b)
{
    return wmtk::utils::TupleInspector::local_vid(a) == wmtk::utils::TupleInspector::local_vid(b) &&
           wmtk::utils::TupleInspector::local_eid(a) == wmtk::utils::TupleInspector::local_eid(b) &&
           wmtk::utils::TupleInspector::local_fid(a) == wmtk::utils::TupleInspector::local_fid(b) &&
           wmtk::utils::TupleInspector::global_cid(a) == wmtk::utils::TupleInspector::global_cid(b);
}

std::vector<PrimitiveType> find_local_switch_sequence_trimesh(
    const Tuple& source,
    const Tuple& target)
{
    // TODO: assert that base_source and base_target use the same global CID

    // circulate
    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;
    if (hash_free_tuple_equality(source, target)) {
        return {};
    }

    cur_tuple = source;

    for (int j = 0; j < 5; ++j) {
        PrimitiveType pt;
        if (j % 2 == 0) {
            pt = PrimitiveType::Vertex;
        } else {
            pt = PrimitiveType::Edge;
        }
        cur_tuple = local_switch_tuple(PrimitiveType::Face, cur_tuple, pt);
        switches.emplace_back(pt);
        if (hash_free_tuple_equality(cur_tuple, target)) {
            return switches;
        }
    }
    throw std::runtime_error(
        "TriMesh switch sequence was unable to find a sequence of switches to match tuples"

        + wmtk::utils::TupleInspector::as_string(source) + "->" +
        wmtk::utils::TupleInspector::as_string(target));
    return switches;
}
std::vector<PrimitiveType> find_local_switch_sequence_edgemesh(
    const Tuple& source,
    const Tuple& target)
{
    if (!hash_free_tuple_equality(source, target)) {
        return std::vector<PrimitiveType>{PrimitiveType::Vertex};
    } else {
        return std::vector<PrimitiveType>{};
    }
    throw std::runtime_error(
        "EdgeMesh switch sequence was unable to find a sequence of switches to match tuples" +
        wmtk::utils::TupleInspector::as_string(source) + "->" +
        wmtk::utils::TupleInspector::as_string(target));
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
    case PrimitiveType::Vertex: return {};
    case PrimitiveType::Tetrahedron:
    default: return {};
    }
}
} // namespace wmtk::multimesh::utils
