#include "tag_intersection.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/TagIntersection.hpp"
#include "internal/TagIntersectionOptions.hpp"

namespace wmtk {
namespace components {
auto gather_attributes(
    io::Cache& cache,
    const Mesh& mesh,
    const internal::TagIntersectionOptions& options)
{
    using TagVec = std::vector<std::tuple<attribute::MeshAttributeHandle, int64_t>>;
    using AttrVec = std::vector<attribute::MeshAttributeHandle>;

    TagVec input_tags;
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
            PrimitiveType::Triangle);

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


    TagVec output_tags;
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
            PrimitiveType::Triangle);

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

    AttrVec pass_through_attributes = base::get_attributes(cache, mesh, options.pass_through);

    return std::make_tuple(input_tags, output_tags, pass_through_attributes);
}

void tag_intersection(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    TagIntersectionOptions options = j.get<TagIntersectionOptions>();

    auto mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    auto [input_tags, output_tags, pass_through_attributes] =
        gather_attributes(cache, mesh, options);

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        for (const auto& [h, v] : input_tags) {
            keeps.emplace_back(h);
        }
        for (const auto& [h, v] : output_tags) {
            keeps.emplace_back(h);
        }
        mesh.clear_attributes();
    }

    std::tie(input_tags, output_tags, pass_through_attributes) =
        gather_attributes(cache, mesh, options);

    switch (mesh_in->top_simplex_type()) {
    case PrimitiveType::Triangle: {
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
