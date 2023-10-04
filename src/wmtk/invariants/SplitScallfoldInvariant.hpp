#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class SplitScallfoldInvariant : public MeshInvariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    SplitScallfoldInvariant(
        const Mesh& m,
        const MeshAttributeHandle<double>& position_handle,
        const MeshAttributeHandle<long>& vertex_tag_handle,
        double threshold_squared,
        long input_tag_value,
        long embedding_tag_value,
        long offset_tag_value);
    using MeshInvariant::MeshInvariant;
    bool before(const Tuple& t) const override;

private:
    const MeshAttributeHandle<double> m_position_handle;
    const MeshAttributeHandle<long> m_vertex_tag_handle;
    double m_threshold_squared;
    long m_input_tag_value;
    long m_embedding_tag_value;
    long m_offset_tag_value;
};
} // namespace wmtk
