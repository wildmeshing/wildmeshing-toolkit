#include "find_local_switch_sequence.hpp"

#include <cassert>
#include <optional>
#include <stdexcept>
#include "local_switch_tuple.hpp"
namespace wmtk::multimesh::utils {


namespace {

bool hash_free_tuple_equality(const Tuple& a, const Tuple& b)
{
    return a.local_vid() == b.local_vid() && a.local_eid() == b.local_eid() &&
           a.local_fid() == b.local_fid() && a.global_cid() == b.global_cid();
}

std::optional<std::vector<PrimitiveType>> find_local_switch_sequence_on_edge(
    const PrimitiveType mesh_pt,
    const Tuple& source,
    const Tuple& target)
{
    assert(mesh_pt >= PrimitiveType::Edge);
    if (hash_free_tuple_equality(source, target)) {
        return std::vector<PrimitiveType>{};
    } else if (hash_free_tuple_equality(
                   local_switch_tuple(mesh_pt, source, PrimitiveType::Vertex),
                   target)) {
        return std::vector<PrimitiveType>{PrimitiveType::Vertex};
    }
    return std::nullopt;
}

std::optional<std::vector<PrimitiveType>> find_local_switch_sequence_on_triangle(
    const PrimitiveType mesh_pt,
    const Tuple& source,
    const Tuple& target)
{
    assert(mesh_pt >= PrimitiveType::Triangle);

    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;
    {
        const auto edge_local_operations =
            find_local_switch_sequence_on_edge(mesh_pt, cur_tuple, target);
        if (edge_local_operations.has_value()) {
            return edge_local_operations.value();
        }
    }
    switches.emplace_back(PrimitiveType::Edge);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        const auto edge_local_operations =
            find_local_switch_sequence_on_edge(mesh_pt, cur_tuple, target);
        if (edge_local_operations.has_value()) {
            switches.insert(
                switches.end(),
                edge_local_operations.value().begin(),
                edge_local_operations.value().end());
            return switches;
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Vertex);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        const auto edge_local_operations =
            find_local_switch_sequence_on_edge(mesh_pt, cur_tuple, target);
        if (edge_local_operations.has_value()) {
            switches.insert(
                switches.end(),
                edge_local_operations.value().begin(),
                edge_local_operations.value().end());
            return switches;
        }
    }
    return std::nullopt;
}

std::optional<std::vector<PrimitiveType>> find_local_switch_sequence_on_tet(
    const PrimitiveType mesh_pt,
    const Tuple& source,
    const Tuple& target)
{
    assert(mesh_pt == PrimitiveType::Tetrahedron);
    Tuple cur_tuple = source;
    std::vector<PrimitiveType> switches;

    {
        const auto triangle_local_operations =
            find_local_switch_sequence_on_triangle(mesh_pt, cur_tuple, target);
        if (triangle_local_operations.has_value()) {
            return triangle_local_operations.value();
        }
    }
    switches.emplace_back(PrimitiveType::Triangle);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        const auto triangle_local_operations =
            find_local_switch_sequence_on_triangle(mesh_pt, cur_tuple, target);
        if (triangle_local_operations.has_value()) {
            switches.insert(
                switches.end(),
                triangle_local_operations.value().begin(),
                triangle_local_operations.value().end());
            return switches;
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Edge);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        const auto triangle_local_operations =
            find_local_switch_sequence_on_triangle(mesh_pt, cur_tuple, target);
        if (triangle_local_operations.has_value()) {
            switches.insert(
                switches.end(),
                triangle_local_operations.value().begin(),
                triangle_local_operations.value().end());
            return switches;
        }
    }

    switches.insert(switches.begin(), PrimitiveType::Vertex);
    cur_tuple = local_switch_tuples(mesh_pt, source, switches);
    {
        const auto triangle_local_operations =
            find_local_switch_sequence_on_triangle(mesh_pt, cur_tuple, target);
        if (triangle_local_operations.has_value()) {
            switches.insert(
                switches.end(),
                triangle_local_operations.value().begin(),
                triangle_local_operations.value().end());
            return switches;
        }
    }
    return std::nullopt;
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
        const auto operations =
            find_local_switch_sequence_on_edge(PrimitiveType::Edge, source, target);
        if (!operations.has_value()) {
            throw std::runtime_error(
                "switch sequence was unable to find a sequence of switches to match tuples");
        }
        return operations.value();
    }
    case PrimitiveType::Triangle: {
        const auto operations =
            find_local_switch_sequence_on_triangle(PrimitiveType::Triangle, source, target);
        if (!operations.has_value()) {
            throw std::runtime_error(
                "switch sequence was unable to find a sequence of switches to match tuples");
        }
        return operations.value();
    }
    case PrimitiveType::Tetrahedron: {
        const auto operations =
            find_local_switch_sequence_on_tet(PrimitiveType::Tetrahedron, source, target);
        if (!operations.has_value()) {
            throw std::runtime_error(
                "switch sequence was unable to find a sequence of switches to match tuples");
        }
        return operations.value();
    }
    case PrimitiveType::Vertex: return {};
    default: return {};
    }
}
} // namespace wmtk::multimesh::utils
