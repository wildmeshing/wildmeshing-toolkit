#pragma once

#include <igl/predicates/predicates.h>
#include <Eigen/Core>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk {

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixXd = MatrixX<double>;
using MatrixXi = MatrixX<int>;
using MatrixXr = MatrixX<Rational>;
using Matrix4d = Eigen::Matrix4d;

template <typename T, int R>
using Vector = Eigen::Matrix<T, R, 1>;
template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

using Vector2d = Vector<double, 2>;
using Vector3d = Vector<double, 3>;
using Vector4d = Vector<double, 4>;
using VectorXd = Vector<double, Eigen::Dynamic>;

using Vector2r = Vector<Rational, 2>;
using Vector3r = Vector<Rational, 3>;

using Vector2i = Vector<int, 2>;
using Vector3i = Vector<int, 3>;
using Vector4i = Vector<int, 4>;
using VectorXi = Vector<int, Eigen::Dynamic>;

inline Vector3r to_rational(const Vector3d& p0)
{
    Vector3r p(p0[0], p0[1], p0[2]);
    return p;
}

inline Vector3d to_double(const Vector3r& p0)
{
    Vector3d p(p0[0].to_double(), p0[1].to_double(), p0[2].to_double());
    return p;
}

inline Vector4d to_homogenuous(const Vector3d& x)
{
    return Vector4d(x[0], x[1], x[2], 1);
}

inline Vector3d from_homogenuous(const Vector4d& x)
{
    return Vector3d(x[0] / x[3], x[1] / x[3], x[2] / x[3]);
}

using V_MAP = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>;
using F_MAP = Eigen::Map<const Eigen::Matrix<int, -1, -1, Eigen::RowMajor>>;

inline void vectors_to_VF(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    MatrixXd& V,
    MatrixXi& F)
{
    V.resize(vertices.size(), 3);
    F.resize(faces.size(), 3);

    for (size_t i = 0; i < V.rows(); i++) {
        V.row(i) = vertices[i];
    }
    for (size_t i = 0; i < F.rows(); i++) {
        F.row(i) = Vector3i(faces[i][0], faces[i][1], faces[i][2]);
    }
}

inline bool
tet_is_inverted(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3)
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(v0, v1, v2, v3);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

inline bool tri_is_inverted(const Vector2d& v0, const Vector2d& v1, const Vector2d& v2)
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(v0, v1, v2);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

/**
 * @brief Convert rational vertex positions to double and check for inversions
 *
 * @return false, if any tet is inverted.
 */
inline bool VF_rational_to_double(const MatrixXr& V_in, const MatrixXi& F, MatrixXd& V_out)
{
    V_out.resizeLike(V_in);
    for (size_t i = 0; i < V_in.size(); ++i) {
        V_out(i) = V_in(i).to_double();
    }

    // check for inversion
    if (F.cols() == 3) {
        // 2D
        for (size_t i = 0; i < F.rows(); ++i) {
            const auto& s = F.row(i);
            if (tri_is_inverted(V_out.row(s[0]), V_out.row(s[1]), V_out.row(s[2]))) {
                return false;
            }
        }
    } else if (F.cols() == 4) {
        // 3D
        for (size_t i = 0; i < F.rows(); ++i) {
            const auto& s = F.row(i);
            if (tet_is_inverted(
                    V_out.row(s[0]),
                    V_out.row(s[1]),
                    V_out.row(s[2]),
                    V_out.row(s[3]))) {
                return false;
            }
        }
    } else {
        log_and_throw_error("Unknown dimension in VF_rational_to_double");
    }

    return true;
}

} // namespace wmtk
