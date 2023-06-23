#pragma once

#include <Eigen/Dense>

#include <type_traits>

namespace wmtk {

template <typename Scalar>
struct BarycentricTriangle
{
    BarycentricTriangle(
        const Eigen::Vector2<Scalar>& a,
        const Eigen::Vector2<Scalar>& b,
        const Eigen::Vector2<Scalar>& c)
    {
        m_a = a;
        m_v0 = b - a;
        m_v1 = c - a;
        m_d00 = m_v0.dot(m_v0);
        m_d01 = m_v0.dot(m_v1);
        m_d11 = m_v1.dot(m_v1);
        m_denom = m_d00 * m_d11 - m_d01 * m_d01;
    }

    Eigen::Vector3<Scalar> get(const Eigen::Vector2<Scalar>& p)
    {
        assert(!is_degenerate());
        const Eigen::Vector2<Scalar> v2 = p - m_a;
        const Scalar d20 = v2.dot(m_v0);
        const Scalar d21 = v2.dot(m_v1);
        Eigen::Vector3<Scalar> coeffs;
        coeffs.y() = (m_d11 * d20 - m_d01 * d21) / m_denom;
        coeffs.z() = (m_d00 * d21 - m_d01 * d20) / m_denom;
        coeffs.x() = Scalar(1.0) - coeffs.y() - coeffs.z();
        return coeffs;
    }

    bool is_degenerate() const
    {
        if constexpr (std::is_floating_point_v<Scalar>) {
            // Scalar type
            if (m_denom < std::numeric_limits<Scalar>().denorm_min()) {
                return true;
            }
        } else {
            // Autodiff type
            if (m_denom.getValue() < std::numeric_limits<typename Scalar::Scalar>().denorm_min()) {
                return true;
            }
        }
        return false;
    }

private:
    // Cached values per triangle
    Eigen::Vector2<Scalar> m_a;
    Eigen::Vector2<Scalar> m_v0;
    Eigen::Vector2<Scalar> m_v1;
    Scalar m_d00;
    Scalar m_d01;
    Scalar m_d11;
    Scalar m_denom;
};

} // namespace wmtk
