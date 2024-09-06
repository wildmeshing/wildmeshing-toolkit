#include "marching.hpp"

#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

#include "MarchingOptions.hpp"
#include "internal/Marching.hpp"

namespace wmtk::components {

void marching(Mesh& mesh, const MarchingOptions& options)
{
    assert(
        mesh.top_simplex_type() == PrimitiveType::Triangle ||
        mesh.top_simplex_type() == PrimitiveType::Tetrahedron);


    attribute::MeshAttributeHandle pos_handle = options.position_handle;
    std::map<PrimitiveType, attribute::MeshAttributeHandle> label_handles = options.label_handles;

    assert(&pos_handle.mesh() == &mesh);

    Marching mc(pos_handle, label_handles, options.input_values, options.output_value);

    for (const auto& [filter_handle, filter_value] : options.edge_filter_handles) {
        mc.add_filter(filter_handle, filter_value);
    }

    mc.add_pass_through(options.pass_through_attributes);
    mc.process();
}

} // namespace wmtk::components
