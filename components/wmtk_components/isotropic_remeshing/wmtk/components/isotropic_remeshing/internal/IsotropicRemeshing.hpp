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
    attribute::MeshAttributeHandle& position,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    const double length,
    const bool lock_boundary,
    const bool use_for_periodic,
    const int64_t iterations,
    const std::vector<attribute::MeshAttributeHandle>& other_positions = {},
    bool update_other_positions = false,
    const std::optional<attribute::MeshAttributeHandle>& position_for_inversion = {});

} // namespace wmtk::components::internal
