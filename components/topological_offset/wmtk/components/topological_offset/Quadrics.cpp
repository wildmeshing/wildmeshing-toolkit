#include "Quadrics.hpp"

#include <Eigen/SVD>

namespace wmtk::components::topological_offset {

Quadrics::Quadrics(const double a, const double b, const double c, const double d)
{
    Vector4d nh(a, b, c, d);
    m_matrix = nh * nh.transpose();
}

Quadrics::Quadrics(const Vector3d& p0, const Vector3d& n)
{
    const double d = -n.transpose() * p0;
    Vector4d nh(n[0], n[1], n[2], d);
    m_matrix = nh * nh.transpose();
}

Quadrics::Quadrics(const Eigen::Ref<const Matrix4d>& matrix)
    : m_matrix(matrix)
{}

Quadrics& Quadrics::operator+=(const Quadrics& other)
{
    m_matrix += other.m_matrix;
    return *this;
}

Quadrics Quadrics::operator+(const Quadrics& other) const
{
    Quadrics q(*this);
    q += other;
    return q;
}

Quadrics Quadrics::operator*(const double scalar) const
{
    Quadrics q(*this);
    q *= scalar;
    return q;
}

Quadrics& Quadrics::operator*=(const double scalar)
{
    m_matrix *= scalar;
    return *this;
}

Vector3d Quadrics::solve(const Eigen::Ref<const Vector3d>& p, const double svd_threshold) const
{
    const Matrix3d A = m_matrix.block(0, 0, 3, 3);
    const Vector3d b = -m_matrix.block(0, 3, 3, 1);

    Eigen::JacobiSVD<Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    svd.setThreshold(svd_threshold);

    // Lindstrom formula for the QEM optimum, regularized toward p along directions the
    // quadric leaves unconstrained ("Out-of-Core Simplification of Large Polygonal Models")
    return p + svd.solve(b - A * p);
}

double Quadrics::squared_distance_to_optimum(
    const Eigen::Ref<const Vector3d>& p,
    const double svd_threshold) const
{
    const Vector3d p_opt = solve(p, svd_threshold);
    return (p_opt - p).squaredNorm();
}

} // namespace wmtk::components::topological_offset
