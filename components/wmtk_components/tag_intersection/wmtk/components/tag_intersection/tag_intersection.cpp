#include "tag_intersection.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/TagIntersection.hpp"
#include "internal/TagIntersectionOptions.hpp"

namespace wmtk {
namespace components {
void tag_intersection(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    TagIntersectionOptions options = j.get<TagIntersectionOptions>();

    auto mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    std::vector<std::tuple<attribute::MeshAttributeHandle, int64_t>> input_tags;
    for (size_t i = 0; i < options.attributes.vertex_labels.size(); ++i) {
        const std::string& name = options.attributes.vertex_labels[i];
        const int64_t& value = options.values.vertex_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.attributes.vertex_labels[i],
            PrimitiveType::Vertex);

        input_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.attributes.edge_labels.size(); ++i) {
        const std::string& name = options.attributes.edge_labels[i];
        const int64_t& value = options.values.edge_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.attributes.edge_labels[i],
            PrimitiveType::Edge);

        input_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.attributes.face_labels.size(); ++i) {
        const std::string& name = options.attributes.face_labels[i];
        const int64_t& value = options.values.face_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.attributes.face_labels[i],
            PrimitiveType::Face);

        input_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.attributes.tetrahedron_labels.size(); ++i) {
        const std::string& name = options.attributes.tetrahedron_labels[i];
        const int64_t& value = options.values.tetrahedron_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.attributes.tetrahedron_labels[i],
            PrimitiveType::Tetrahedron);

        input_tags.emplace_back(std::make_tuple(handle, value));
    }


    std::vector<std::tuple<attribute::MeshAttributeHandle, int64_t>> output_tags;
    for (size_t i = 0; i < options.output_attributes.vertex_labels.size(); ++i) {
        const std::string& name = options.output_attributes.vertex_labels[i];
        const int64_t& value = options.output_values.vertex_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.output_attributes.vertex_labels[i],
            PrimitiveType::Vertex);

        output_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.output_attributes.edge_labels.size(); ++i) {
        const std::string& name = options.output_attributes.edge_labels[i];
        const int64_t& value = options.output_values.edge_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.output_attributes.edge_labels[i],
            PrimitiveType::Edge);

        output_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.output_attributes.face_labels.size(); ++i) {
        const std::string& name = options.output_attributes.face_labels[i];
        const int64_t& value = options.output_values.face_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.output_attributes.face_labels[i],
            PrimitiveType::Face);

        output_tags.emplace_back(std::make_tuple(handle, value));
    }
    for (size_t i = 0; i < options.output_attributes.tetrahedron_labels.size(); ++i) {
        const std::string& name = options.output_attributes.tetrahedron_labels[i];
        const int64_t& value = options.output_values.tetrahedron_values[i];

        auto handle = mesh.get_attribute_handle<int64_t>(
            options.output_attributes.tetrahedron_labels[i],
            PrimitiveType::Tetrahedron);

        output_tags.emplace_back(std::make_tuple(handle, value));
    }

    switch (mesh_in->top_simplex_type()) {
    case PrimitiveType::Face: {
        TriMesh& m = static_cast<TriMesh&>(*mesh_in);
        wmtk::components::TagIntersection tag_intersection;
        tag_intersection.compute_intersection(m, input_tags, output_tags);
        break;
    }
    case PrimitiveType::Tetrahedron: {
        TetMesh& m = static_cast<TetMesh&>(*mesh_in);
        wmtk::components::TagIntersection tag_intersection;
        tag_intersection.compute_intersection(m, input_tags, output_tags);
        break;
    }
    default: {
        log_and_throw_error(
            "Works only for faces and tetrahedrons, error-type: {}",
            mesh_in->top_simplex_type());
        break;
    }
    }

    cache.write_mesh(*mesh_in, options.output);
}
} // namespace components
} // namespace wmtk
