#include "c1_utils.hpp"
#include "clough_tocher/clough_tocher_matrices.hpp"

namespace wmtk::components::c1_simplification {

double
orient2d_sign(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
}

bool point_in_tri(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    double s1 = orient2d_sign(p, a, b);
    double s2 = orient2d_sign(p, b, c);
    double s3 = orient2d_sign(p, c, a);

    // if same sign then true
    double eps = 1e-10;
    return (s1 >= -eps && s2 >= -eps && s3 >= -eps) || (s1 <= eps && s2 <= eps && s3 <= eps);
}

Eigen::Vector2d barycentric_coord_in_tri(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    Eigen::Vector2d vec0 = c - a;
    Eigen::Vector2d vec1 = b - a;
    Eigen::Vector2d vec2 = p - a;

    double d00 = vec0.dot(vec0);
    double d01 = vec0.dot(vec1);
    double d11 = vec1.dot(vec1);
    double d20 = vec2.dot(vec0);
    double d21 = vec2.dot(vec1);

    double denom = d00 * d11 - d01 * d01;

    if (denom < 1e-12) {
        wmtk::logger().warn("nearly degenerated triangle");
    }

    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    return Eigen::Vector2d(u, v);
}


/////////////////////////////
//// clough tocher patch ////
/////////////////////////////

// CT const matrices
const std::array<Eigen::Matrix<double, 3, 3>, 3> m_CTtri_bounds = CT_subtri_bound_matrices();
const std::array<Eigen::Matrix<double, 10, 12>, 3> m_CT_matrices = CT_subtri_matrices();

int triangle_ind(const double& u, const double& v, const double& w)
{
    int idx = -1;
    for (int i = 0; i < 3; ++i) {
        if (m_CTtri_bounds[i](0, 0) * u + m_CTtri_bounds[i](0, 1) * v +
                    m_CTtri_bounds[i](0, 2) * w >=
                -1e-7 &&
            m_CTtri_bounds[i](1, 0) * u + m_CTtri_bounds[i](1, 1) * v +
                    m_CTtri_bounds[i](1, 2) * w >=
                -1e-7 &&
            m_CTtri_bounds[i](2, 0) * u + m_CTtri_bounds[i](2, 1) * v +
                    m_CTtri_bounds[i](2, 2) * w >=
                -1e-7) {
            idx = i;
            break;
        }
    }

    assert(idx > -1);
    return idx;
}


Eigen::Matrix<double, 10, 1> monomial_basis_eval(const double& u, const double& v, const double& w)
{
    Eigen::Matrix<double, 10, 1> monomial_basis_values;
    monomial_basis_values(0, 0) = w * w * w; // w3
    monomial_basis_values(1, 0) = v * w * w; // vw2
    monomial_basis_values(2, 0) = v * v * w; // v2w
    monomial_basis_values(3, 0) = v * v * v; // v3
    monomial_basis_values(4, 0) = u * w * w; // uw2
    monomial_basis_values(5, 0) = u * v * w; // uvw
    monomial_basis_values(6, 0) = u * v * v; // uv2
    monomial_basis_values(7, 0) = u * u * w; // u2w
    monomial_basis_values(8, 0) = u * u * v; // u2v
    monomial_basis_values(9, 0) = u * u * u; // u3

    return monomial_basis_values;
}

Eigen::Matrix<double, 10, 2> monomial_basis_grad(const double& u, const double& v)
{
    Eigen::Matrix<double, 10, 2> monomial_basis_grads;
    // u
    monomial_basis_grads(0, 0) = -3 * (-u - v + 1.0) * (-u - v + 1.0);
    monomial_basis_grads(1, 0) = v * (2 * u + 2 * v - 2.0);
    monomial_basis_grads(2, 0) = -v * v;
    monomial_basis_grads(3, 0) = 0;
    monomial_basis_grads(4, 0) = u * (2 * u + 2 * v - 2.0) + (-u - v + 1.0) * (-u - v + 1.0);
    monomial_basis_grads(5, 0) = -u * v + v * (-u - v + 1.0);
    monomial_basis_grads(6, 0) = v * v;
    monomial_basis_grads(7, 0) = -u * u + 2 * u * (-u - v + 1.0);
    monomial_basis_grads(8, 0) = 2 * u * v;
    monomial_basis_grads(9, 0) = 3 * u * u;

    // v
    monomial_basis_grads(0, 1) = -3 * (-u - v + 1.0) * (-u - v + 1.0);
    monomial_basis_grads(1, 1) = v * (2 * u + 2 * v - 2.0) + (-u - v + 1.0) * (-u - v + 1.0);
    monomial_basis_grads(2, 1) = -v * v + 2 * v * (-u - v + 1.0);
    monomial_basis_grads(3, 1) = 3 * v * v;
    monomial_basis_grads(4, 1) = u * (2 * u + 2 * v - 2.0);
    monomial_basis_grads(5, 1) = -u * v + u * (-u - v + 1.0);
    monomial_basis_grads(6, 1) = 2 * u * v;
    monomial_basis_grads(7, 1) = -u * u;
    monomial_basis_grads(8, 1) = u * u;
    monomial_basis_grads(9, 1) = 0;

    return monomial_basis_grads;
}


Eigen::Vector3d CT_eval(const double& u, const double& v, const Eigen::Matrix<double, 12, 3>& dofs)
{
    const double w = 1.0 - u - v;
    int idx = triangle_ind(u, v, w);

    Eigen::Matrix<double, 10, 1> bb_vector = monomial_basis_eval(u, v, w);
    Eigen::Vector3d val;

    Eigen::Matrix<double, 10, 3> m_CT_coeffs_subtri = m_CT_matrices[idx] * dofs;

    val = m_CT_coeffs_subtri.transpose() * bb_vector;

    return val;
}

Eigen::Matrix<double, 3, 2>
CT_grad(const double& u, const double& v, const Eigen::Matrix<double, 12, 3>& dofs)
{
    const double w = 1.0 - u - v;
    int idx = triangle_ind(u, v, w);

    Eigen::Matrix<double, 10, 2> bb_vector = monomial_basis_grad(u, v);
    Eigen::Matrix<double, 3, 2> grad;

    Eigen::Matrix<double, 10, 3> m_CT_coeffs_subtri = m_CT_matrices[idx] * dofs;

    grad = m_CT_coeffs_subtri.transpose() * bb_vector;

    return grad;
}

} // namespace wmtk::components::c1_simplification