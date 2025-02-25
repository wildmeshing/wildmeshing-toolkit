#include "rgb.hpp"
#include <wmtk/Mesh.hpp>
#include "RGB1.hpp"
#include "RGB2.hpp"


namespace wmtk::components::rgb {

const attribute::MeshAttributeHandle& RGB::position_handle() const
{
    return options.position_attribute_handle;
}

void rgb(const RGBOptions& options)
{
    std::unique_ptr<RGB> rgb;
    auto& mesh = options.position_attribute_handle.mesh();
    switch (mesh.top_cell_dimension()) {
    case 1: rgb = std::make_unique<RGB1>(options); break;
    case 2: rgb = std::make_unique<RGB2>(options); break;
    default: assert(false);
    }
    rgb->run();
}

} // namespace wmtk::components::rgb

