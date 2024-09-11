#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::multimesh::utils {

// Maps the tuple source according to the operation sequence
// std::vector<PrimitiveType> operations where operations satisfies
// base_target = switch_tuples(base_source, operations)
Tuple local_switch_tuple(
    PrimitiveType mesh_primitive_type,
    const Tuple& source,
    PrimitiveType primitive_type);


// Performs a sequence of switch_tuple operations in the order specified in sequence.
// in debug mode this will assert a failure, in release this will return a null tuple
//#if defined(__cpp_concepts)
//template <std::forward_iterator ContainerType>
//#else
template <typename ContainerType>
//#endif
Tuple local_switch_tuples(
    PrimitiveType mesh_primitive_type,
    const Tuple& tuple,
    const ContainerType& sequence);
// annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
Tuple local_switch_tuples(
    PrimitiveType mesh_primitive_type,
    const Tuple& tuple,
    const std::initializer_list<PrimitiveType>& sequence);


// IMPLEMENTATION of above declaration
//#if defined(__cpp_concepts)
//template <std::forward_iterator ContainerType>
//#else
template <typename ContainerType>
//#endif
Tuple local_switch_tuples(
    PrimitiveType mesh_primitive_type,
    const Tuple& tuple,
    const ContainerType& sequence)
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    for (const PrimitiveType primitive : sequence) {
        r = local_switch_tuple(mesh_primitive_type, r, primitive);
    }
    return r;
}

} // namespace wmtk::multimesh::utils
