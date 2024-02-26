#include "mesh_decimation.hpp"
#include "internal/MeshDecimation.hpp"
#include "internal/MeshDecimationOptions.hpp"
#include "wmtk/components/base/get_attributes.hpp"


namespace wmtk::components {

void mesh_decimation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    MeshDecimationOptions options = j.get<MeshDecimationOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    Mesh& mesh = *mesh_in;
    auto pass_through_attributes = base::get_attributes(cache, mesh, options.pass_through);
    auto original_attributes = base::get_attributes(cache, mesh, options.attributes);
    auto cell_constrait_tag_handle = mesh.get_attribute_handle<int64_t>(
        options.cell_constrait_tag_name,
        mesh.top_simplex_type());

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.insert(keeps.end(), original_attributes.begin(), original_attributes.end());
        mesh.clear_attributes(keeps);
    }

    MeshDecimation md(mesh, cell_constrait_tag_handle, options.target_len, pass_through_attributes);

    md.process();

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.insert(keeps.end(), original_attributes.begin(), original_attributes.end());
        mesh.clear_attributes(keeps);
    }

    cache.write_mesh(*mesh_in, options.output);
}
} // namespace wmtk::components