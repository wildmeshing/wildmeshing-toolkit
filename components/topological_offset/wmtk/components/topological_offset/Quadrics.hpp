#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::topological_offset {

/**
 * @brief A quadric error metric (QEM), represented as a 4x4 matrix built from weighted plane
 * samples. Adapted from
 * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/utils/Quadrics.hpp,
 * stripped down to the plain linear-algebra core: sampling quadrics from a BVH and mesh
 * attributes is left to the caller (see Smooth.cpp) instead of being built into this class.
 */
class Quadrics
{
public:
    /**
     * @brief construct a quadric from the plane equation ax+by+cz+d=0
     */
    Quadrics(const double a, const double b, const double c, const double d);

    /**
     * @brief construct a quadric from a point on the plane and its normal
     */
    Quadrics(const Vector3d& p0, const Vector3d& n);

    Quadrics(const Eigen::Ref<const Matrix4d>& matrix);

    Quadrics(const Quadrics& other) = default;

    Quadrics& operator+=(const Quadrics& other);
    Quadrics operator+(const Quadrics& other) const;
    Quadrics operator*(const double scalar) const;
    Quadrics& operator*=(const double scalar);

    /**
     * @brief solve for the position minimizing the quadric error, regularized toward p along
     * directions the quadric leaves unconstrained (relevant for quadrics built from a single
     * plane, whose A matrix is singular)
     */
    Vector3d solve(const Eigen::Ref<const Vector3d>& p) const;

    double squared_distance_to_optimum(const Eigen::Ref<const Vector3d>& p) const;

private:
    Matrix4d m_matrix = Matrix4d::Zero();
};

} // namespace wmtk::components::topological_offset
