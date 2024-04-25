#pragma once

#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/Triangle2DTo3DMapping.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include "RGBSplit.hpp"

namespace wmtk::operations::composite {

class RGBSplitWithPositionOptimization : public Operation
{
public:
    RGBSplitWithPositionOptimization(
        Mesh& m,
        attribute::MeshAttributeHandle& uv_handle,
        std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> mapping_ptr,
        attribute::MeshAttributeHandle& triangle_rgb_state_handle,
        attribute::MeshAttributeHandle& edge_rgb_state_handle);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline RGBSplit& split() { return m_rgb_split; }
    inline AttributesUpdateWithFunction& attribute_update() { return m_attribute_update; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    Eigen::Vector2d get_best_uv(
        const Eigen::Vector2d& edge_uv0,
        const Eigen::Vector2d& edge_uv1,
        const Eigen::Vector2d& top_uv,
        const std::optional<Eigen::Vector2d>& btm_uv_opt) const;

private:
    RGBSplit m_rgb_split;
    AttributesUpdateWithFunction m_attribute_update;
    attribute::MeshAttributeHandle m_uv_handle;
    std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> m_mapping_ptr;
};
} // namespace wmtk::operations::composite
