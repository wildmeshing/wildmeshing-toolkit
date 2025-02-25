
#pragma once
#include "RGBOptions.hpp"
#include "wmtk/attribute/MeshAttributes.hpp"


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

    const attribute::MeshAttributeHandle& position_handle() const;
};


void rgb(const RGBOptions& options);
} // namespace wmtk::components::rgb
