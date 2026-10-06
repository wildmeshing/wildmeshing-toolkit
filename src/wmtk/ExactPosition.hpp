#pragma once

#include <wmtk/Types.hpp>

#include <memory>

namespace wmtk {

/**
 * @brief A vertex's exact (rational) position, stored only while it differs from the vertex's
 * double position.
 *
 * The optimizer meshes keep every vertex position twice: as doubles, which everything reads,
 * and as rationals, which only the exact predicates read and only while a vertex cannot be
 * rounded. Once a vertex is rounded its exact position IS its double position -- the code keeps
 * the two equal by construction -- so the rational copy carries no information, yet it cost a
 * Vector of GMP rationals per vertex slot: 32 bytes inline per coordinate plus heap limbs,
 * roughly 290 bytes per 3D vertex and most of a vertex slot's footprint.
 *
 * So the rational position lives behind a pointer that is null whenever it would equal the
 * double one, and value() rebuilds it from the double position on demand. Reads see exactly
 * the value they saw before; only vertices that are genuinely not representable in doubles
 * (and the few that are mid-operation) pay for the rational storage.
 *
 * Copying copies the stored position. There are deliberately no move operations: moving one of
 * these copies it, as moving the Vector of Rationals it replaces did (Rational has no move
 * constructor), so attribute slots moved from during consolidation keep their contents.
 */
template <class VecR, class VecD>
class ExactPosition
{
public:
    ExactPosition() = default;
    ExactPosition(const ExactPosition& o)
        : m_p(o.m_p ? std::make_unique<VecR>(*o.m_p) : nullptr)
    {}
    ExactPosition& operator=(const ExactPosition& o)
    {
        if (this == &o) return *this;
        if (!o.m_p) {
            m_p.reset();
        } else if (m_p) {
            *m_p = *o.m_p;
        } else {
            m_p = std::make_unique<VecR>(*o.m_p);
        }
        return *this;
    }

    /// The exact position of a vertex whose double position is `posf`.
    VecR value(const VecD& posf) const { return m_p ? *m_p : to_rational(posf); }

    /// Whether coordinate `k` equals `r`, without materialising the whole position.
    bool coord_equals(int k, const VecD& posf, const Rational& r) const
    {
        return m_p ? (*m_p)[k] == r : Rational(posf[k]) == r;
    }

    /// Store an exact position that may differ from the double one.
    void store(const VecR& p)
    {
        if (m_p) {
            *m_p = p;
        } else {
            m_p = std::make_unique<VecR>(p);
        }
    }

    /// The exact position is the double one: drop the stored copy.
    void clear() { m_p.reset(); }

    bool stored() const { return static_cast<bool>(m_p); }

private:
    std::unique_ptr<VecR> m_p;
};

} // namespace wmtk
