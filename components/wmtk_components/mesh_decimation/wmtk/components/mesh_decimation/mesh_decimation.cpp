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
    Mesh& mesh = static_cast<Mesh&>(*mesh_in);
    auto pass_through_attributes = base::get_attributes(cache, mesh, options.pass_through);

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        mesh.clear_attributes(keeps);
    }

    MeshDecimation md(
        mesh,
        options.cell_constrait_tag_name,
        options.constrait_value,
        options.target_len,
        pass_through_attributes);

    md.process();

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        mesh.clear_attributes(keeps);
    }

    cache.write_mesh(*mesh_in, options.output);
}
} // namespace wmtk::components