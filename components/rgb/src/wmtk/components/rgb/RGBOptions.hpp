#pragma once
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::components::rgb {
struct RGBOptions
{
    wmtk::attribute::MeshAttributeHandle position_attribute_handle;
    // the threshold for which edges must be shorter than
    double max_edge_length;
};
} // namespace wmtk::components::rgb
