#include "marching.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "internal/Marching.hpp"
#include "internal/MarchingOptions.hpp"

namespace wmtk::components {

auto gather_attributes(io::Cache& cache, const Mesh& mesh, const internal::MarchingOptions& options)
{
    attribute::MeshAttributeHandle vertex_tag_handle =
        mesh.get_attribute_handle<int64_t>(options.attributes.vertex_label, PrimitiveType::Vertex);

    std::vector<attribute::MeshAttributeHandle> filter_labels;
    for (const std::string& name : options.attributes.filter_labels) {
        attribute::MeshAttributeHandle handle =
            mesh.get_attribute_handle<int64_t>(name, PrimitiveType::Edge);
        filter_labels.emplace_back(handle);
    }

    auto pass_through_attributes = utils::get_attributes(cache, mesh, options.pass_through);

    return std::make_tuple(vertex_tag_handle, filter_labels, pass_through_attributes);
}

auto get_marching_attributes(io::Cache& cache, Mesh& mesh, const internal::MarchingOptions& options)
{
    std::optional<attribute::MeshAttributeHandle> edge_tag_handle;
    std::optional<attribute::MeshAttributeHandle> face_tag_handle;

    if (!options.attributes.edge_label.empty()) {
        assert(options.attributes.edge_label.size() == 1);
        const std::string& edge_label_name = options.attributes.edge_label[0];
        if (mesh.has_attribute<int64_t>(edge_label_name, PrimitiveType::Edge)) {
            edge_tag_handle =
                mesh.get_attribute_handle<int64_t>(edge_label_name, PrimitiveType::Edge);
        } else {
            edge_tag_handle =
                mesh.register_attribute<int64_t>(edge_label_name, PrimitiveType::Edge, 1);
        }
    }

    if (!options.attributes.face_label.empty()) {
        assert(options.attributes.face_label.size() == 1);
        const std::string& face_label_name = options.attributes.face_label[0];
        if (mesh.has_attribute<int64_t>(face_label_name, PrimitiveType::Triangle)) {
            face_tag_handle =
                mesh.get_attribute_handle<int64_t>(face_label_name, PrimitiveType::Triangle);
        } else {
            face_tag_handle =
                mesh.register_attribute<int64_t>(face_label_name, PrimitiveType::Triangle, 1);
        }
    }

    return std::make_tuple(edge_tag_handle, face_tag_handle);
}

void marching(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    MarchingOptions options = j.get<MarchingOptions>();

    // input
    auto x = options.input;
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);
    assert(options.input_values.size() == 2);

    auto [vertex_tag_handle, filter_labels, pass_through_attributes] =
        gather_attributes(cache, mesh, options);

    auto [edge_tag_handle, face_tag_handle] = get_marching_attributes(cache, mesh, options);

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(vertex_tag_handle);
        if (edge_tag_handle.has_value()) {
            keeps.emplace_back(edge_tag_handle.value());
        }
        if (face_tag_handle.has_value()) {
            keeps.emplace_back(face_tag_handle.value());
        }
        keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
        mesh.clear_attributes(keeps);
    }

    std::tie(vertex_tag_handle, filter_labels, pass_through_attributes) =
        gather_attributes(cache, mesh, options);

    std::tie(edge_tag_handle, face_tag_handle) = get_marching_attributes(cache, mesh, options);

    assert(
        mesh.top_simplex_type() == PrimitiveType::Triangle ||
        mesh.top_simplex_type() == PrimitiveType::Tetrahedron);

    if (edge_tag_handle.has_value()) {
        if (face_tag_handle.has_value()) {
            Marching mc(
                mesh,
                vertex_tag_handle,
                edge_tag_handle.value(),
                face_tag_handle.value(),
                options.input_values,
                options.output_value,
                options.weight,
                filter_labels,
                options.filter_values,
                pass_through_attributes);
            mc.process();
        } else {
            Marching mc(
                mesh,
                vertex_tag_handle,
                edge_tag_handle.value(),
                options.input_values,
                options.output_value,
                options.weight,
                filter_labels,
                options.filter_values,
                pass_through_attributes);
            mc.process();
        }
    } else {
        Marching mc(
            mesh,
            vertex_tag_handle,
            options.input_values,
            options.output_value,
            options.weight,
            filter_labels,
            options.filter_values,
            pass_through_attributes);
        mc.process();
    }

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(vertex_tag_handle);
        if (edge_tag_handle.has_value()) {
            keeps.emplace_back(edge_tag_handle.value());
        }
        if (face_tag_handle.has_value()) {
            keeps.emplace_back(face_tag_handle.value());
        }
        keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
        mesh.clear_attributes(keeps);
    }

    cache.write_mesh(*mesh_in, options.output);
}

} // namespace wmtk::components
