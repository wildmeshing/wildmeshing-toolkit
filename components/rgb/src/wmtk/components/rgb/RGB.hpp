
#pragma once
#include "RGBOptions.hpp"


namespace wmtk::components::multimesh {
    class MeshCollection;
}

namespace wmtk::components::rgb {


class RGB
{
public:
    RGB(const RGBOptions& opts)
        : options(opts)
    {}
    virtual ~RGB();
    virtual void run() = 0;

protected:
    RGBOptions options;

    const attribute::MeshAttributeHandle& position_handle(
    components::multimesh::MeshCollection& meshes
            ) const;
};


void rgb(components::multimesh::MeshCollection& mc, const RGBOptions& options);
} // namespace wmtk::components::rgb
