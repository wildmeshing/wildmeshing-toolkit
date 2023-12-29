#include "SimplexGetter.hpp"

#include <wmtk/simplex/faces_single_dimension.hpp>

namespace wmtk::function::utils {

template <typename T>
std::tuple<std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>>, long> get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<T>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex_in,
    const std::optional<wmtk::Tuple>& vertex_marker)
{
    const Simplex simplex =
        mesh.is_ccw(simplex_in.tuple())
            ? simplex_in
            : Simplex(simplex_in.primitive_type(), mesh.switch_vertex(simplex_in.tuple()));

    assert(mesh.is_ccw(simplex.tuple()));
    const std::vector<Tuple> faces =
        wmtk::simplex::faces_single_dimension_tuples(mesh, simplex, primitive_type);


    std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>> ret;
    ret.reserve(faces.size());
    long vertex_marker_index = -1;

    std::transform(
        faces.begin(),
        faces.end(),
        std::back_inserter(ret),
        [&](const Tuple& face_tuple) {
            auto value = accessor.const_vector_attribute(face_tuple).eval();
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

template std::tuple<std::vector<Eigen::Matrix<char, Eigen::Dynamic, 1>>, long>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<char>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<long, Eigen::Dynamic, 1>>, long>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<long>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>>, long>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<double>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<Rational, Eigen::Dynamic, 1>>, long>
get_simplex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<Rational>& accessor,
    const PrimitiveType primitive_type,
    const wmtk::Simplex& simplex,
    const std::optional<wmtk::Tuple>& vertex_marker);
} // namespace wmtk::function::utils
