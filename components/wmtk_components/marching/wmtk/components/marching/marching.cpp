#include "marching.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/components/base/resolve_path.hpp>

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

    auto pass_through_attributes = base::get_attributes(cache, mesh, options.pass_through);

    return std::make_tuple(vertex_tag_handle, filter_labels, pass_through_attributes);
}

auto get_marching_attributes(io::Cache& cache, Mesh& mesh, const internal::MarchingOptions& options)
{
    std::vector<attribute::MeshAttributeHandle> marching_handles;

    if (!options.marching_edge_tag_name.empty() &&
        mesh.has_attribute<int64_t>(options.marching_edge_tag_name[0], PrimitiveType::Edge)) {
        marching_handles.emplace_back(mesh.get_attribute_handle<int64_t>(
            options.marching_edge_tag_name[0],
            PrimitiveType::Edge));
    } else if (!options.marching_edge_tag_name.empty()) {
        auto handle = mesh.register_attribute<int64_t>(
            options.marching_edge_tag_name[0],
            PrimitiveType::Edge,
            1);
        marching_handles.emplace_back(handle);
    } else {
        if (mesh.has_attribute<int64_t>("marching_edge_tag", PrimitiveType::Edge)) {
            auto handle =
                mesh.get_attribute_handle<int64_t>("marching_edge_tag", PrimitiveType::Edge);
            marching_handles.emplace_back(handle);
        } else {
            auto handle =
                mesh.register_attribute<int64_t>("marching_edge_tag", PrimitiveType::Edge, 1);
            marching_handles.emplace_back(handle);
        }
    }

    if (!options.marching_face_tag_name.empty() &&
        mesh.has_attribute<int64_t>(options.marching_face_tag_name[0], PrimitiveType::Triangle)) {
        marching_handles.emplace_back(mesh.get_attribute_handle<int64_t>(
            options.marching_face_tag_name[0],
            PrimitiveType::Triangle));
    } else if (!options.marching_face_tag_name.empty()) {
        auto handle = mesh.register_attribute<int64_t>(
            options.marching_face_tag_name[0],
            PrimitiveType::Triangle,
            1);
        marching_handles.emplace_back(handle);
    } else {
        if (mesh.has_attribute<int64_t>("marching_face_tag", PrimitiveType::Triangle)) {
            auto handle =
                mesh.get_attribute_handle<int64_t>("marching_face_tag", PrimitiveType::Triangle);
            marching_handles.emplace_back(handle);
        } else {
            auto handle =
                mesh.register_attribute<int64_t>("marching_face_tag", PrimitiveType::Triangle, 1);
            marching_handles.emplace_back(handle);
        }
    }


    assert(marching_handles.size() == 2);

    return std::make_tuple(marching_handles[0], marching_handles[1]);
}

void marching(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
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

    auto [marching_edge_tag_handle, marching_face_tag_handle] =
        get_marching_attributes(cache, mesh, options);

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(vertex_tag_handle);
        keeps.emplace_back(marching_edge_tag_handle);
        keeps.emplace_back(marching_face_tag_handle);
        keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
        mesh.clear_attributes(keeps);
    }

    std::tie(vertex_tag_handle, filter_labels, pass_through_attributes) =
        gather_attributes(cache, mesh, options);

    std::tie(marching_edge_tag_handle, marching_face_tag_handle) =
        get_marching_attributes(cache, mesh, options);

    switch (mesh.top_cell_dimension()) {
    case 2:
    case 3: {
        // Marching mc(mesh, vertex_tags, options.output_vertex_tag, edge_filter_tags);
        Marching mc(
            mesh,
            marching_edge_tag_handle,
            marching_face_tag_handle,
            vertex_tag_handle,
            options.input_values,
            options.output_value,
            options.weight,
            filter_labels,
            options.filter_values,
            pass_through_attributes);
        mc.process();
    } break;
    default: throw std::runtime_error("dimension setting error!"); break;
    }

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(vertex_tag_handle);
        keeps.emplace_back(marching_edge_tag_handle);
        keeps.emplace_back(marching_face_tag_handle);
        keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
        mesh.clear_attributes(keeps);
    }

    cache.write_mesh(*mesh_in, options.output);
}

} // namespace wmtk::components
