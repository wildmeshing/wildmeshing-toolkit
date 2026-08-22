#pragma once

#include <wmtk/envelope/Envelope.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * CONTAINMENT-ONLY composites over per-tag boundary envelopes. BOTH DIMENSIONS.
 *
 * Both classes override exactly the four virtual queries -- is_outside(triangle3) and
 * is_outside(point3), virtual on wmtk::Envelope, plus is_outside(segment2) and
 * is_outside(point2), virtual on SampleEnvelope for this purpose -- which are the only calls
 * the containment paths make: surface_triangle_is_outside() (TetOptimizerMesh.h) and
 * surface_segment_is_outside() (TriOptimizerMesh.h) for every split/collapse/swap check and
 * the sanity sweep, smoothing_containment_envelope() consumers (SmoothVertex.hpp, both
 * dimensions), and the one hardcoded m_envelope point check in collapse_edge_before
 * (TetOptimizerMeshCollapse.cpp).
 *
 * THE INVARIANT THAT KEEPS THEM SAFE: a composite must never reach a caller of the
 * NON-virtual SampleEnvelope queries -- nearest_point, nearest_point_feature,
 * squared_distance, or the edge/2D is_outside overloads. Those would statically bind to
 * the base subobject, whose BVH was never built, and dereference null. The only such
 * callers are fed by smoothing_energy_envelope() (the pull: SmoothVertex.hpp and
 * EnvelopeEnergy), and that hook returns a single real member envelope by contract --
 * the most-violated one for a junction vertex -- never a composite.
 */

/// Inside means inside EVERY member: the tube intersection. This is what pins a junction
/// simplex to the junction -- within eps of each boundary it lies on -- and what replaced
/// the dedicated input-complex segment/point envelopes: a wire or isolated point of the
/// complex only arises where two or more selected tags meet, so the intersection of those
/// tags' tubes holds it in place.
class IntersectionEnvelope final : public SampleEnvelope
{
public:
    explicit IntersectionEnvelope(std::vector<std::shared_ptr<SampleEnvelope>> members)
        : m_members(std::move(members))
    {}

    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const override
    {
        for (const auto& e : m_members) {
            if (e->is_outside(tris)) return true;
        }
        return false;
    }

    bool is_outside(const Eigen::Vector3d& pts) const override
    {
        for (const auto& e : m_members) {
            if (e->is_outside(pts)) return true;
        }
        return false;
    }

    bool is_outside(const std::array<Eigen::Vector2d, 2>& edge) const override
    {
        for (const auto& e : m_members) {
            if (e->is_outside(edge)) return true;
        }
        return false;
    }

    bool is_outside(const Eigen::Vector2d& pts) const override
    {
        for (const auto& e : m_members) {
            if (e->is_outside(pts)) return true;
        }
        return false;
    }

    const std::vector<std::shared_ptr<SampleEnvelope>>& members() const { return m_members; }

private:
    std::vector<std::shared_ptr<SampleEnvelope>> m_members;
};

/// Inside means inside ANY member: the tube union, which is exactly what the old fused
/// region-boundary envelope answered for a point. Assigned to the inherited m_envelope so
/// the legacy "may this surface vertex land on a non-surface position" point check keeps
/// its semantics with zero shared-code changes.
class UnionEnvelope final : public SampleEnvelope
{
public:
    explicit UnionEnvelope(std::vector<std::shared_ptr<SampleEnvelope>> members)
        : m_members(std::move(members))
    {}

    bool is_outside(const std::array<Eigen::Vector3d, 3>& tris) const override
    {
        for (const auto& e : m_members) {
            if (!e->is_outside(tris)) return false;
        }
        return true;
    }

    bool is_outside(const Eigen::Vector3d& pts) const override
    {
        for (const auto& e : m_members) {
            if (!e->is_outside(pts)) return false;
        }
        return true;
    }

    bool is_outside(const std::array<Eigen::Vector2d, 2>& edge) const override
    {
        for (const auto& e : m_members) {
            if (!e->is_outside(edge)) return false;
        }
        return true;
    }

    bool is_outside(const Eigen::Vector2d& pts) const override
    {
        for (const auto& e : m_members) {
            if (!e->is_outside(pts)) return false;
        }
        return true;
    }

private:
    std::vector<std::shared_ptr<SampleEnvelope>> m_members;
};

} // namespace wmtk::components::topological_offset
