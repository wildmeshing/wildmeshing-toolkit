#pragma once

#include <Eigen/Core>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
template <typename T>
class SimplexInversionInvariant : public Invariant
{
public:
    SimplexInversionInvariant(
        const Mesh& m,
        const TypedAttributeHandle<T>& coordinate,
        bool inverted = false);
    using Invariant::Invariant;

    /**
     *  we assume with local vid order (v0,v1,v2,v3) has positive volume (orient3d(v0,v1,v2,v3)>0)
     a CLOCKWISE input tuple will have the same orientation as the local vid order
     */
    bool after(const std::vector<Tuple>&, const std::vector<Tuple>& top_dimension_tuples_after)
        const override;

private:
    bool is_oriented(const T& p0, const T& p1) const;
    bool is_oriented(const Eigen::Ref<const Vector1<T>>& p0, const Eigen::Ref<const Vector1<T>>& p1)
        const;
    bool is_oriented(
        const Eigen::Ref<const Vector2<T>>& p0,
        const Eigen::Ref<const Vector2<T>>& p1,
        const Eigen::Ref<const Vector2<T>>& p2) const;
    bool is_oriented(
        const Eigen::Ref<const Vector3<T>>& p0,
        const Eigen::Ref<const Vector3<T>>& p1,
        const Eigen::Ref<const Vector3<T>>& p2,
        const Eigen::Ref<const Vector3<T>>& p3) const;

    const TypedAttributeHandle<T> m_coordinate_handle;
    bool m_inverted = false;
};

} // namespace wmtk
