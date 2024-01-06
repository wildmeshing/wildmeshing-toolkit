#include "marching.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>

#include "internal/Marching.hpp"
#include "internal/MarchingOptions.hpp"

namespace wmtk::components {

void marching(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    MarchingOptions options = j.get<MarchingOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    assert(options.input_values.size() == 2);

    attribute::MeshAttributeHandle vertex_tag_handle =
        mesh.get_attribute_handle<int64_t>(options.attributes.vertex_label, PrimitiveType::Vertex);

    std::vector<attribute::MeshAttributeHandle> filter_labels;
    for (const std::string& name : options.attributes.filter_labels) {
        attribute::MeshAttributeHandle handle =
            mesh.get_attribute_handle<int64_t>(name, PrimitiveType::Edge);
        filter_labels.emplace_back(handle);
    }

    auto pass_through_attributes = base::get_attributes(mesh, options.pass_through);

    switch (mesh.top_cell_dimension()) {
    case 2:
    case 3: {
        // Marching mc(mesh, vertex_tags, options.output_vertex_tag, edge_filter_tags);
        Marching mc(
            mesh,
            vertex_tag_handle,
            options.input_values,
            options.output_value,
            filter_labels,
            options.filter_values,
            pass_through_attributes);
        mc.process();
    } break;
    default: throw std::runtime_error("dimension setting error!"); break;
    }

    cache.write_mesh(*mesh_in, options.output);
}

} // namespace wmtk::components
