#include "mesh_decimation.hpp"
#include "internal/MeshDecimation.hpp"
#include "internal/MeshDecimationOptions.hpp"


namespace wmtk::components {

// compute the length relative to the bounding box diagonal
double compute_target_len(
    const attribute::MeshAttributeHandle& pos_handle,
    const double decimation_relative_len_factor)
{
    auto pos = pos_handle.mesh().create_const_accessor<double>(pos_handle);
    const auto vertices = pos_handle.mesh().get_all(PrimitiveType::Vertex);
    double avg = 0;
    double cnt = 0;
    for (const Tuple edge : pos_handle.mesh().get_all(PrimitiveType::Edge)) {
        double len =
            (pos.vector_attribute(edge) -
             pos.vector_attribute(pos_handle.mesh().switch_tuple(edge, PrimitiveType::Vertex)))
                .norm();
        avg = avg * cnt / (cnt + 1) + len / (cnt + 1);
        cnt++;
    }

    return avg * decimation_relative_len_factor;
}

void mesh_decimation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    MeshDecimationOptions options = j.get<MeshDecimationOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    options.pass_through;
    // vector<>
    Mesh& mesh = static_cast<Mesh&>(*mesh_in);
    std::vector<attribute::MeshAttributeHandle> pass_though_attributes;
    for (int i = 0; i < options.pass_through.size(); ++i) {
        // base get attribute...
    }

    MeshDecimation md(
        mesh,
        options.cell_constrait_tag_name,
        options.constrait_value,
        1,
        pass_though_attributes);

    // // clear attributes
    // {
    //     std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    //     keeps.emplace_back(vertex_tag_handle);
    //     keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
    //     mesh.clear_attributes(keeps);
    // }

    // std::tie(vertex_tag_handle, filter_labels, pass_through_attributes) =
    //     gather_attributes(cache, mesh, options);

    // switch (mesh.top_cell_dimension()) {
    // case 2:
    // case 3: {
    //     // Marching mc(mesh, vertex_tags, options.output_vertex_tag, edge_filter_tags);
    //     Marching mc(
    //         mesh,
    //         vertex_tag_handle,
    //         options.input_values,
    //         options.output_value,
    //         filter_labels,
    //         options.filter_values,
    //         pass_through_attributes);
    //     mc.process();
    // } break;
    // default: throw std::runtime_error("dimension setting error!"); break;
    // }

    // // clear attributes
    // {
    //     std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    //     keeps.emplace_back(vertex_tag_handle);
    //     keeps.insert(keeps.end(), filter_labels.begin(), filter_labels.end());
    //     mesh.clear_attributes(keeps);
    // }

    cache.write_mesh(*mesh_in, options.output);
}
} // namespace wmtk::components