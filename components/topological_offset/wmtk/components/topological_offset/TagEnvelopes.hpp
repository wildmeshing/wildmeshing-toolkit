#pragma once

#include <wmtk/envelope/Envelope.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * Containment-only composites over per-tag boundary envelopes, in both dimensions.
 *
 * Both classes override exactly the four virtual queries -- is_outside(triangle3) and
 * is_outside(point3) on wmtk::Envelope, is_outside(segment2) and is_outside(point2) on
 * SampleEnvelope -- which are the only calls the containment paths make.
 *
 * A composite must never reach a caller of the non-virtual SampleEnvelope queries:
 * nearest_point, nearest_point_feature, squared_distance, or the edge/2D is_outside overloads.
 * Those bind statically to the base subobject, whose BVH was never built, and dereference null.
 * Such callers are fed by smoothing_energy_envelope(), and that hook returns a single real member
 * envelope by contract -- the most-violated one for a junction vertex -- never a composite.
 */

/// Inside means inside every member: the tube intersection. This is what pins a junction simplex
/// to the junction, within eps of each boundary it lies on, and what holds a wire or isolated
/// point of the input complex in place, since those only arise where two or more selected tags
/// meet.
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

/// Inside means inside any member: the tube union. Assigned to the inherited m_envelope, so the
/// shared engine's "may this surface vertex land on a non-surface position" point check keeps its
/// semantics with no changes there.
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
