#include "SimplexGetter.hpp"

#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>

namespace wmtk::function::utils {

template <typename T>
std::tuple<std::vector<std::decay_t<typename attribute::ConstMapResult<T>>>, int64_t>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::Accessor<T>& accessor,
    const PrimitiveType primitive_type,
    const simplex::Simplex& simplex_in,
    const std::optional<wmtk::Tuple>& vertex_marker)
{
    std::vector<Tuple> faces;
    if (primitive_type == PrimitiveType::Vertex) {
        faces = mesh.orient_vertices(simplex_in.tuple());
    } else {
        const simplex::Simplex simplex =
            mesh.is_ccw(simplex_in.tuple())
                ? simplex_in
                : simplex::Simplex(
                      mesh,
                      simplex_in.primitive_type(),
                      mesh.switch_tuple(simplex_in.tuple(), PrimitiveType::Vertex));

        assert(mesh.is_ccw(simplex.tuple()));
        faces = wmtk::simplex::faces_single_dimension_tuples(mesh, simplex, primitive_type);
    }

    std::vector<std::decay_t<typename attribute::ConstMapResult<T>>> ret;
    ret.reserve(faces.size());
    int64_t vertex_marker_index = -1;

    std::transform(
        faces.begin(),
        faces.end(),
        std::back_inserter(ret),
        [&](const Tuple& face_tuple) {
            auto value = accessor.const_vector_attribute(face_tuple);
            return value;
        });

    // if we have a vertex marker

    if (vertex_marker.has_value()) {
        const auto& variable_tuple = vertex_marker.value();
        for (size_t i = 0; i < faces.size(); ++i) {
            if (wmtk::simplex::utils::SimplexComparisons::equal(
                    mesh,
                    primitive_type,
                    faces[i],
                    variable_tuple)) {
                vertex_marker_index = i;
                break;
            }
        }
        assert(vertex_marker_index >= 0);
    }
    return {ret, vertex_marker_index};
}

template std::tuple<std::vector<std::decay_t<typename attribute::ConstMapResult<char>>>, int64_t>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::Accessor<char>& accessor,
    const PrimitiveType primitive_type,
    const simplex::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<std::decay_t<typename attribute::ConstMapResult<int64_t>>>, int64_t>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::Accessor<int64_t>& accessor,
    const PrimitiveType primitive_type,
    const simplex::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<std::decay_t<typename attribute::ConstMapResult<double>>>, int64_t>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::Accessor<double>& accessor,
    const PrimitiveType primitive_type,
    const simplex::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::
    tuple<std::vector<std::decay_t<typename attribute::ConstMapResult<Rational>>>, int64_t>
    get_simplex_attributes(
        const Mesh& mesh,
        const wmtk::attribute::Accessor<Rational>& accessor,
        const PrimitiveType primitive_type,
        const simplex::Simplex& simplex,
        const std::optional<wmtk::Tuple>& vertex_marker);
} // namespace wmtk::function::utils
