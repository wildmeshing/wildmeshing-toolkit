#pragma once

#include <map>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::components {

struct MarchingOptions
{
    /**
     * vertex positions (double)
     */
    attribute::MeshAttributeHandle position_handle;
    /**
     * Labels for inside, outside, and marching surface (int64_t).
     * Must at least contain vertex labels. Edge and face labels are optional. The top simplex type
     * must not be labeled. If there is a label on the top simplex type (e.g. tetrahedra) add it as
     * "pass through attribute".
     */
    std::map<PrimitiveType, attribute::MeshAttributeHandle> label_handles;
    /**
     * Filters (int64_t) on edge labels. If filter exist, only edges that are INSIDE that filter are
     * considered. Thus, multiple filteres can be combined to get a more strict selection.
     */
    std::vector<std::pair<attribute::MeshAttributeHandle, int64_t>> edge_filter_handles;
    /**
     * Any other attribute goes here. They are handled with the default attribute behavior.
     */
    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;

    /**
     * The label values (int64_t) in between the marching surface should be placed. If only one
     * value is specified, marching is performed in between this value and any other. There must not
     * be more than two values specified.
     */
    std::vector<int64_t> input_values;
    /**
     * The output label value (int 64_t) is assigned to the marching surface in all primitive types
     * for which label handles were specified.
     */
    int64_t output_value;
};

} // namespace wmtk::components