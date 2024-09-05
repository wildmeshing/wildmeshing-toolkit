#include "marching.hpp"

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "internal/Marching.hpp"
#include "internal/MarchingOptions.hpp"

namespace wmtk::components {

auto gather_attributes(const Mesh& mesh, const MarchingOptions& options)
{
    attribute::MeshAttributeHandle vertex_tag_handle =
        mesh.get_attribute_handle<int64_t>(options.attributes.vertex_label, PrimitiveType::Vertex);

    std::vector<attribute::MeshAttributeHandle> filter_labels;
    for (const std::string& name : options.attributes.filter_labels) {
        attribute::MeshAttributeHandle handle =
            mesh.get_attribute_handle<int64_t>(name, PrimitiveType::Edge);
        filter_labels.emplace_back(handle);
    }

    auto pass_through_attributes = utils::get_attributes(mesh, options.pass_through);

    return std::make_tuple(vertex_tag_handle, filter_labels, pass_through_attributes);
}

auto get_marching_attributes(Mesh& mesh, const MarchingOptions& options)
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

void marching(Mesh& mesh, const nlohmann::json& j)
{
    MarchingOptions options = j.get<MarchingOptions>();

    assert(options.input_values.size() == 2);

    auto [vertex_tag_handle, filter_labels, pass_through_attributes] =
        gather_attributes(mesh, options);

    auto [edge_tag_handle, face_tag_handle] = get_marching_attributes(mesh, options);

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
        gather_attributes(mesh, options);

    std::tie(edge_tag_handle, face_tag_handle) = get_marching_attributes(mesh, options);

    assert(
        mesh.top_simplex_type() == PrimitiveType::Triangle ||
        mesh.top_simplex_type() == PrimitiveType::Tetrahedron);

    auto pos_handle =
        mesh.get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);

    std::map<PrimitiveType, attribute::MeshAttributeHandle> label_handles;
    label_handles[PrimitiveType::Vertex] = vertex_tag_handle;
    if (edge_tag_handle.has_value()) {
        label_handles[PrimitiveType::Edge] = edge_tag_handle.value();
    }
    if (face_tag_handle.has_value()) {
        label_handles[PrimitiveType::Triangle] = face_tag_handle.value();
    }

    Marching mc(pos_handle, label_handles, options.input_values, options.output_value);
    for (size_t i = 0; i < filter_labels.size(); ++i) {
        mc.add_filter(filter_labels[i], options.filter_values[i]);
    }
    mc.add_pass_through(pass_through_attributes);
    mc.process();

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
}

} // namespace wmtk::components
