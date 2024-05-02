#pragma once

#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/Triangle2DTo3DMapping.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include "RGBSplit.hpp"

namespace wmtk::operations::composite {

class PositionOptimizationOnEdge : public Operation
{
public:
    PositionOptimizationOnEdge(
        Mesh& m,
        attribute::MeshAttributeHandle& uv_handle,
        std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> mapping_ptr);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    inline AttributesUpdateWithFunction& attribute_update() { return m_attribute_update; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;


private:
    AttributesUpdateWithFunction m_attribute_update;
    attribute::MeshAttributeHandle m_uv_handle;
    std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> m_mapping_ptr;
};
} // namespace wmtk::operations::composite
