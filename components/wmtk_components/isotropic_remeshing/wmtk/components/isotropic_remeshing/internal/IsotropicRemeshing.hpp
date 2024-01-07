#pragma once

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>

#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>

namespace wmtk::components::internal {

void isotropic_remeshing(
    TriMesh& mesh,
    attribute::MeshAttributeHandle& position,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    const double length,
    const bool lock_boundary,
    const int64_t iterations,
    const std::shared_ptr<attribute::MeshAttributeHandle>& position_for_inversion);

} // namespace wmtk::components::internal
