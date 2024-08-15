#include "best_action_to_simplex.hpp"
#include <cassert>
#include "subgroup/subgroup_transformations.hpp"
namespace wmtk::autogen {
namespace {

template <size_t DartSize, size_t SimplexSize>
auto run(
    PrimitiveType mesh_type,
    int8_t orientation,
    PrimitiveType target_simplex_type,
    int8_t target_simplex_index,
    const int8_t (&actions)[DartSize][SimplexSize],
    const int8_t (&max_preservations)[DartSize][SimplexSize]) -> std::tuple<int8_t, PrimitiveType>
{
    const int8_t simplex_index = local_simplex_index(mesh_type, target_simplex_type, orientation);
    const int8_t relative_index =
        relative_simplex_index(mesh_type, target_simplex_type, simplex_index, target_simplex_index);

    return std::tuple(
        actions[orientation][relative_index],
        get_primitive_type_from_id(max_preservations[orientation][relative_index]));
}
} // namespace

auto best_action_to_simplex(
    PrimitiveType mesh_type,
    int8_t orientation,
    PrimitiveType target_simplex_type,
    int8_t target_simplex_index) -> std::tuple<int8_t, PrimitiveType>
{
    switch (mesh_type) {
    case PrimitiveType::Triangle: {
        switch (target_simplex_type) {
        case PrimitiveType::Edge: {
            return run(
                mesh_type,
                orientation,
                target_simplex_type,
                target_simplex_index,
                subgroup::subdart_preserving_action_2_1,
                subgroup::max_subdart_preservation_dimension_2_1);
        }
        case PrimitiveType::Triangle:
        case PrimitiveType::Tetrahedron:
        case PrimitiveType::Vertex:
        default: break;
        }
    }
    case PrimitiveType::Tetrahedron: {
        switch (target_simplex_type) {
        case PrimitiveType::Edge: {
            return run(
                mesh_type,
                orientation,
                target_simplex_type,
                target_simplex_index,
                subgroup::subdart_preserving_action_3_1,
                subgroup::max_subdart_preservation_dimension_3_1);
        }
        case PrimitiveType::Triangle: {
            return run(
                mesh_type,
                orientation,
                target_simplex_type,
                target_simplex_index,
                subgroup::subdart_preserving_action_3_2,
                subgroup::max_subdart_preservation_dimension_3_2);
        }
        case PrimitiveType::Tetrahedron:
        case PrimitiveType::Vertex:
        default: break;
        }
    }
    case PrimitiveType::Edge:
    case PrimitiveType::Vertex:
    default: break;
    }
    assert(false);
    return {};
}
} // namespace wmtk::autogen
