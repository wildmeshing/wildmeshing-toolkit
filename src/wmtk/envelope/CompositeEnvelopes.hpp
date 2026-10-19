#pragma once

#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/Logger.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace wmtk {

/**
 * Containment-only composites over several envelopes, in both dimensions.
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
 *
 * Moved here from topological_offset, which built them for its per-tag tubes, when tetwild and
 * triwild began holding each tracked simplex to the envelopes of its own inputs (InputEnvelopes
 * below).
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

/**
 * @brief One envelope per input, and the containment and the pull of an INPUT SET (bit i =
 * input i; see SurfaceTagAttributes::input_set()).
 *
 * The invariant it serves: a tracked simplex stays within eps of every input in its set.
 * containment() is therefore that input's own envelope for a one-input set, and the
 * IntersectionEnvelope over the set's members for a larger one -- containment only, by the
 * contract above. pull() is the real member envelope a vertex is drawn toward, the set's
 * most-violated member at the vertex's position, as topological_offset pulls a junction vertex:
 * the containment check then enforces the whole set, one real envelope pulls at a time.
 *
 * An empty set has no containment: a tracked simplex carrying one is an insertion or
 * propagation defect, and throws rather than being held by some envelope it never covered.
 */
class InputEnvelopes
{
public:
    /// Member i is input i's envelope; null for an input no tracked simplex can cover (one with
    /// no triangle). Not while the operations run: it drops the intersections built so far.
    void set(std::vector<std::shared_ptr<SampleEnvelope>> members)
    {
        if (members.size() > 16) {
            log_and_throw_error(
                "{} inputs; a tracked simplex records its inputs in 16 bits, so at most 16 are "
                "supported",
                members.size());
        }
        m_members = std::move(members);
        m_intersections.clear();
    }

    size_t size() const { return m_members.size(); }
    bool empty() const { return m_members.empty(); }
    const std::shared_ptr<SampleEnvelope>& member(size_t i) const { return m_members[i]; }

    std::shared_ptr<SampleEnvelope> containment(const uint16_t set) const
    {
        check(set);
        if ((set & (set - 1)) == 0) return m_members[lowest_bit(set)];
        std::lock_guard<std::mutex> lock(m_mutex);
        std::shared_ptr<SampleEnvelope>& e = m_intersections[set];
        if (!e) {
            std::vector<std::shared_ptr<SampleEnvelope>> members;
            for (size_t i = 0; i < m_members.size(); ++i) {
                if (set & (1u << i)) members.push_back(m_members[i]);
            }
            e = std::make_shared<IntersectionEnvelope>(std::move(members));
        }
        return e;
    }

    /// The member of `set` farthest from `p` (Vector2d or Vector3d). Ties go to the lower input.
    template <typename Point>
    std::shared_ptr<SampleEnvelope> pull(const uint16_t set, const Point& p) const
    {
        check(set);
        if ((set & (set - 1)) == 0) return m_members[lowest_bit(set)];
        std::shared_ptr<SampleEnvelope> best;
        double worst = -1.;
        for (size_t i = 0; i < m_members.size(); ++i) {
            if (!(set & (1u << i))) continue;
            const double d2 = m_members[i]->squared_distance(p);
            if (d2 > worst) {
                worst = d2;
                best = m_members[i];
            }
        }
        return best;
    }

private:
    void check(const uint16_t set) const
    {
        if (set == 0) {
            log_and_throw_error(
                "a tracked simplex has an empty input set: the insertion or an operation did "
                "not carry it");
        }
        if (set >> m_members.size()) {
            log_and_throw_error(
                "input set {:#x} names an input beyond the {} that have envelopes",
                set,
                m_members.size());
        }
    }
    static size_t lowest_bit(const uint16_t set)
    {
        size_t i = 0;
        while (!(set & (1u << i))) ++i;
        return i;
    }

    std::vector<std::shared_ptr<SampleEnvelope>> m_members;
    /// Built on first use, under the mutex: the operations ask concurrently.
    mutable std::map<uint16_t, std::shared_ptr<SampleEnvelope>> m_intersections;
    mutable std::mutex m_mutex;
};

} // namespace wmtk
