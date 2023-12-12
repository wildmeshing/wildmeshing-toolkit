#include "SimplexGetter.hpp"

#include <wmtk/simplex/faces_single_dimension.hpp>

namespace wmtk::function::utils {

template <typename T>
std::tuple<std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>>, size_t> get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<T>& accessor,
    const attribute::MeshAttributeHandle<T>& variable_attribute_handle,
    const wmtk::Tuple& simplex,
    const PrimitiveType& domain_simplex_type,
    const std::optional<wmtk::Tuple>& vertex_marker)
{
    const PrimitiveType primitive_type = variable_attribute_handle.primitive_type();
    const std::vector<Tuple> faces = wmtk::simplex::faces_single_dimension_tuples(
        mesh,
        Simplex(domain_simplex_type, simplex),
        primitive_type);


    std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1>> ret;
    ret.reserve(faces.size());
    size_t vertex_marker_index = -1;

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

template std::tuple<std::vector<Eigen::Matrix<char, Eigen::Dynamic, 1>>, size_t>
get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<char>& accessor,
    const attribute::MeshAttributeHandle<char>& variable_attribute_handle,
    const wmtk::Tuple& simplex,
    const PrimitiveType& domain_simplex_type,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<long, Eigen::Dynamic, 1>>, size_t>
get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<long>& accessor,
    const attribute::MeshAttributeHandle<long>& variable_attribute_handle,
    const wmtk::Tuple& simplex,
    const PrimitiveType& domain_simplex_type,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>>, size_t>
get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<double>& accessor,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle,
    const wmtk::Tuple& simplex,
    const PrimitiveType& domain_simplex_type,
    const std::optional<wmtk::Tuple>& vertex_marker);

template std::tuple<std::vector<Eigen::Matrix<Rational, Eigen::Dynamic, 1>>, size_t>
get_simplex_vertex_attributes(
    const Mesh& mesh,
    const wmtk::attribute::ConstAccessor<Rational>& accessor,
    const attribute::MeshAttributeHandle<Rational>& variable_attribute_handle,
    const wmtk::Tuple& simplex,
    const PrimitiveType& domain_simplex_type,
    const std::optional<wmtk::Tuple>& vertex_marker);
} // namespace wmtk::function::utils
