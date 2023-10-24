#include "find_local_switch_sequence.hpp"
#include <spdlog/spdlog.h>
#include <cassert>
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

std::pair<bool, std::vector<PrimitiveType>> find_local_switch_sequence_on_edge(
    const Tuple& source,
    const Tuple& target,
    const PrimitiveType mesh_pt)
{
    assert(mesh_pt >= PrimitiveType::Edge);
    if (hash_free_tuple_equality(source, target)) {
        return {true, std::vector<PrimitiveType>{}};
    } else if (hash_free_tuple_equality(
                   local_switch_tuple(mesh_pt, source, PrimitiveType::Vertex),
                   target)) {
        return {true, std::vector<PrimitiveType>{PrimitiveType::Vertex}};
    }
    return {false, std::vector<PrimitiveType>{}};
}

std::pair<bool, std::vector<PrimitiveType>> find_local_switch_sequence_on_triangle(
    const Tuple& source,
    const Tuple& target,
    const PrimitiveType mesh_pt)
{
    assert(mesh_pt >= PrimitiveType::Face);

    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;
    {
        auto [success, edge_local_operations] =
            find_local_switch_sequence_on_edge(cur_tuple, target, mesh_pt);
        if (success) {
            return {true, edge_local_operations};
        }
    }
    switches.emplace_back(PrimitiveType::Edge);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        auto [success, edge_local_operations] =
            find_local_switch_sequence_on_edge(cur_tuple, target, mesh_pt);
        if (success) {
            switches.insert(
                switches.end(),
                edge_local_operations.begin(),
                edge_local_operations.end());
            return {true, switches};
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Vertex);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        auto [success, edge_local_operations] =
            find_local_switch_sequence_on_edge(cur_tuple, target, mesh_pt);
        if (success) {
            switches.insert(
                switches.end(),
                edge_local_operations.begin(),
                edge_local_operations.end());
            return {true, switches};
        }
    }
    return {false, std::vector<PrimitiveType>{}};
}

std::pair<bool, std::vector<PrimitiveType>> find_local_switch_sequence_on_tet(
    const Tuple& source,
    const Tuple& target,
    const PrimitiveType mesh_pt)
{
    // TODO: test it in the future
    throw "not tested yet";
    assert(mesh_pt == PrimitiveType::Tetrahedron);
    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;

    {
        auto [success, triangle_local_operations] =
            find_local_switch_sequence_on_triangle(cur_tuple, target, mesh_pt);
        if (success) {
            return {true, triangle_local_operations};
        }
    }
    switches.emplace_back(PrimitiveType::Face);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        auto [success, triangle_local_operations] =
            find_local_switch_sequence_on_triangle(cur_tuple, target, mesh_pt);
        if (success) {
            switches.insert(
                switches.end(),
                triangle_local_operations.begin(),
                triangle_local_operations.end());
            return {true, switches};
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Edge);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        auto [success, triangle_local_operations] =
            find_local_switch_sequence_on_triangle(cur_tuple, target, mesh_pt);
        if (success) {
            switches.insert(
                switches.end(),
                triangle_local_operations.begin(),
                triangle_local_operations.end());
            return {true, switches};
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Vertex);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        auto [success, triangle_local_operations] =
            find_local_switch_sequence_on_triangle(cur_tuple, target, mesh_pt);
        if (success) {
            switches.insert(
                switches.end(),
                triangle_local_operations.begin(),
                triangle_local_operations.end());
            return {true, switches};
        }
    }
    return {false, std::vector<PrimitiveType>{}};
}

} // namespace

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
std::vector<PrimitiveType>
find_local_switch_sequence(const Tuple& source, const Tuple& target, PrimitiveType primitive_type)
{
    switch (primitive_type) {
    case PrimitiveType::Edge: {
        auto [success, operations] =
            find_local_switch_sequence_on_edge(source, target, PrimitiveType::Edge);
        if (!success) {
            throw "switch sequence was unable to find a sequence of switches to match tuples";
        }
        return operations;
    }
    case PrimitiveType::Face: {
        auto [success, operations] =
            find_local_switch_sequence_on_triangle(source, target, PrimitiveType::Face);
        if (!success) {
            throw "switch sequence was unable to find a sequence of switches to match tuples";
        }
        return operations;
    }
    case PrimitiveType::Tetrahedron: {
        auto [success, operations] =
            find_local_switch_sequence_on_tet(source, target, PrimitiveType::Tetrahedron);
        if (!success) {
            throw "switch sequence was unable to find a sequence of switches to match tuples";
        }
        return operations;
    }
    case PrimitiveType::Vertex: return {};
    default: return {};
    }
}
} // namespace wmtk::multimesh::utils
