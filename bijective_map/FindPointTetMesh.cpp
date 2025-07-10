#include "FindPointTetMesh.hpp"
#include <iostream>
#include <wmtk/utils/orient.hpp>

std::pair<int, Eigen::Vector4d> findTetContainingPoint(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const Eigen::Vector3d& p,
    double tolerance)
{
    // Initialize result: -1 means no tetrahedron contains the point
    std::pair<int, Eigen::Vector4d> result = {-1, Eigen::Vector4d::Zero()};

    // Iterate through all tetrahedra
    for (int i = 0; i < T.rows(); ++i) {
        // Extract the vertices of the current tetrahedron
        Eigen::Vector3d v0 = V.row(T(i, 0));
        Eigen::Vector3d v1 = V.row(T(i, 1));
        Eigen::Vector3d v2 = V.row(T(i, 2));
        Eigen::Vector3d v3 = V.row(T(i, 3));

        // Construct the matrix for barycentric coordinate calculation
        Eigen::Matrix4d M;
        M << v0(0), v1(0), v2(0), v3(0), v0(1), v1(1), v2(1), v3(1), v0(2), v1(2), v2(2), v3(2),
            1.0, 1.0, 1.0, 1.0; // The last row is constant 1

        // Construct the right-hand side vector for the system
        Eigen::Vector4d rhs;
        rhs << p(0), p(1), p(2), 1.0;

        // Solve for barycentric coordinates
        Eigen::Vector4d barycentric = M.colPivHouseholderQr().solve(rhs);

        // Check if the barycentric coordinates are within the valid range [0, 1]
        if ((barycentric.array() >= -tolerance).all() &&
            (barycentric.array() <= 1.0 + tolerance).all()) {
            // If the point is inside the tetrahedron, update the result and return
            result.first = i; // Index of the containing tetrahedron
            result.second = barycentric; // Barycentric coordinates
            return result;
        }
    }

    // Return the default result (-1, Zero) if the point is not found in any tetrahedron
    return result;
}

std::pair<int, Eigen::Vector4d> findTetContainingPointOrient3d(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const Eigen::Vector3d& p)
{
    // Initialize result: -1 means no tetrahedron contains the point
    std::pair<int, Eigen::Vector4d> result = {-1, Eigen::Vector4d::Zero()};

    // Iterate through all tetrahedra
    for (int i = 0; i < T.rows(); ++i) {
        // Extract the vertices of the current tetrahedron
        Eigen::Vector3d v0 = V.row(T(i, 0));
        Eigen::Vector3d v1 = V.row(T(i, 1));
        Eigen::Vector3d v2 = V.row(T(i, 2));
        Eigen::Vector3d v3 = V.row(T(i, 3));

        // Use orient3d to test if point p is on the same side of each face as the opposite vertex
        // A point is inside a tetrahedron if it has the same orientation as the opposite vertex
        // for all four faces of the tetrahedron

        // Face 0: v1, v2, v3 (opposite vertex is v0)
        int orient_face0 = wmtk::utils::wmtk_orient3d(v1, v2, v3, p);
        int orient_v0 = wmtk::utils::wmtk_orient3d(v1, v2, v3, v0);

        // Face 1: v0, v2, v3 (opposite vertex is v1)
        int orient_face1 = wmtk::utils::wmtk_orient3d(v0, v2, v3, p);
        int orient_v1 = wmtk::utils::wmtk_orient3d(v0, v2, v3, v1);

        // Face 2: v0, v1, v3 (opposite vertex is v2)
        int orient_face2 = wmtk::utils::wmtk_orient3d(v0, v1, v3, p);
        int orient_v2 = wmtk::utils::wmtk_orient3d(v0, v1, v3, v2);

        // Face 3: v0, v1, v2 (opposite vertex is v3)
        int orient_face3 = wmtk::utils::wmtk_orient3d(v0, v1, v2, p);
        int orient_v3 = wmtk::utils::wmtk_orient3d(v0, v1, v2, v3);

        // Check if point has same orientation as opposite vertex for all faces
        // This means the point is inside the tetrahedron
        bool inside = (orient_face0 * orient_v0 >= 0) && (orient_face1 * orient_v1 >= 0) &&
                      (orient_face2 * orient_v2 >= 0) && (orient_face3 * orient_v3 >= 0);

        if (inside) {
            // If the point is inside the tetrahedron, compute barycentric coordinates
            // using volume ratios for numerical stability

            // Helper function to compute signed volume of tetrahedron using determinant
            auto computeSignedVolume = [](const Eigen::Vector3d& a,
                                          const Eigen::Vector3d& b,
                                          const Eigen::Vector3d& c,
                                          const Eigen::Vector3d& d) -> double {
                Eigen::Matrix3d mat;
                mat.row(0) = b - a;
                mat.row(1) = c - a;
                mat.row(2) = d - a;
                return mat.determinant() / 6.0;
            };

            // Compute volume of the original tetrahedron
            double total_volume = computeSignedVolume(v0, v1, v2, v3);

            // Compute barycentric coordinates using volume ratios
            Eigen::Vector4d barycentric;
            barycentric[0] =
                computeSignedVolume(p, v1, v2, v3) / total_volume; // Volume opposite to v0
            barycentric[1] =
                computeSignedVolume(v0, p, v2, v3) / total_volume; // Volume opposite to v1
            barycentric[2] =
                computeSignedVolume(v0, v1, p, v3) / total_volume; // Volume opposite to v2
            barycentric[3] =
                computeSignedVolume(v0, v1, v2, p) / total_volume; // Volume opposite to v3

            // Check for negative barycentric coordinates (should not happen for point inside
            // tetrahedron)
            bool has_negative = false;
            for (int j = 0; j < 4; j++) {
                if (barycentric[j] < 0) {
                    has_negative = true;
                    break;
                }
            }

            if (has_negative) {
                std::cout << "Warning: Negative barycentric coordinates detected for point ("
                          << p.transpose() << ") in tetrahedron " << i << std::endl;
                std::cout << "Barycentric coordinates: " << barycentric.transpose() << std::endl;
                std::cout
                    << "This indicates inconsistency between orient3d test and volume calculation"
                    << std::endl;
                continue; // Skip this tetrahedron
            }

            // Normalize to ensure sum equals 1 (numerical stability)
            double sum = barycentric.sum();
            barycentric /= sum;

            result.first = i;
            result.second = barycentric;

            std::cout << "Point (" << p.transpose() << ") is contained in tetrahedron " << i
                      << " with barycentric coordinates " << barycentric.transpose() << std::endl;

            return result;
        }
    }

    // Print error message if point is not found in any tetrahedron
    std::cout << "Warning: Point (" << p.transpose()
              << ") is not contained in any tetrahedron in findTetContainingPointOrient3d"
              << std::endl;

    // Return the default result (-1, Zero) if the point is not found in any tetrahedron
    return result;
}

std::pair<int, Eigen::Matrix<wmtk::Rational, 4, 1>> findTetContainingPointRational(
    const Eigen::Matrix<wmtk::Rational, Eigen::Dynamic, 3>& V,
    const Eigen::MatrixXi& T,
    const Eigen::Matrix<wmtk::Rational, 3, 1>& p)
{
    // Initialize result: -1 means no tetrahedron contains the point
    std::pair<int, Eigen::Matrix<wmtk::Rational, 4, 1>> result = {
        -1,
        Eigen::Matrix<wmtk::Rational, 4, 1>::Zero()};

    // Helper function to compute signed volume of tetrahedron using determinant
    auto computeSignedVolumeRational =
        [](const Eigen::Matrix<wmtk::Rational, 3, 1>& a,
           const Eigen::Matrix<wmtk::Rational, 3, 1>& b,
           const Eigen::Matrix<wmtk::Rational, 3, 1>& c,
           const Eigen::Matrix<wmtk::Rational, 3, 1>& d) -> wmtk::Rational {
        Eigen::Matrix<wmtk::Rational, 3, 3> mat;
        mat.row(0) = b - a;
        mat.row(1) = c - a;
        mat.row(2) = d - a;
        return mat.determinant() / wmtk::Rational(6);
    };

    // Iterate through all tetrahedra
    for (int i = 0; i < T.rows(); ++i) {
        // Extract the vertices of the current tetrahedron
        Eigen::Matrix<wmtk::Rational, 3, 1> v0 = V.row(T(i, 0));
        Eigen::Matrix<wmtk::Rational, 3, 1> v1 = V.row(T(i, 1));
        Eigen::Matrix<wmtk::Rational, 3, 1> v2 = V.row(T(i, 2));
        Eigen::Matrix<wmtk::Rational, 3, 1> v3 = V.row(T(i, 3));

        // Compute volume of the original tetrahedron
        wmtk::Rational total_volume = computeSignedVolumeRational(v0, v1, v2, v3);

        // Skip degenerate tetrahedra (zero volume)
        if (total_volume == wmtk::Rational(0)) {
            continue;
        }

        // Compute barycentric coordinates using volume ratios
        Eigen::Matrix<wmtk::Rational, 4, 1> barycentric;
        barycentric[0] =
            computeSignedVolumeRational(p, v1, v2, v3) / total_volume; // Volume opposite to v0
        barycentric[1] =
            computeSignedVolumeRational(v0, p, v2, v3) / total_volume; // Volume opposite to v1
        barycentric[2] =
            computeSignedVolumeRational(v0, v1, p, v3) / total_volume; // Volume opposite to v2
        barycentric[3] =
            computeSignedVolumeRational(v0, v1, v2, p) / total_volume; // Volume opposite to v3

        // Check if the barycentric coordinates are within the valid range [0, 1]
        bool all_non_negative = true;
        bool all_less_than_one = true;

        for (int j = 0; j < 4; ++j) {
            if (barycentric(j) < wmtk::Rational(0)) {
                all_non_negative = false;
                break;
            }
            if (barycentric(j) > wmtk::Rational(1)) {
                all_less_than_one = false;
                break;
            }
        }

        if (all_non_negative && all_less_than_one) {
            // Normalize to ensure sum equals 1 (numerical stability)
            wmtk::Rational sum = barycentric.sum();
            barycentric /= sum;

            // If the point is inside the tetrahedron, update the result and return
            result.first = i; // Index of the containing tetrahedron
            result.second = barycentric; // Barycentric coordinates
            return result;
        }
    }

    // Return the default result (-1, Zero) if the point is not found in any tetrahedron
    return result;
}

// Helper functions for rational conversions
Eigen::Matrix<wmtk::Rational, Eigen::Dynamic, 3> toRationalMatrix(const Eigen::MatrixXd& V)
{
    Eigen::Matrix<wmtk::Rational, Eigen::Dynamic, 3> V_rational(V.rows(), 3);
    for (int i = 0; i < V.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            V_rational(i, j) = wmtk::Rational(V(i, j));
        }
    }
    return V_rational;
}

Eigen::Matrix<wmtk::Rational, 3, 1> toRationalVector(const Eigen::Vector3d& p)
{
    Eigen::Matrix<wmtk::Rational, 3, 1> p_rational;
    for (int i = 0; i < 3; ++i) {
        p_rational(i) = wmtk::Rational(p(i));
    }
    return p_rational;
}

Eigen::Vector4d toDoubleBarycentric(const Eigen::Matrix<wmtk::Rational, 4, 1>& rational_bc)
{
    Eigen::Vector4d bc_double;
    for (int i = 0; i < 4; ++i) {
        bc_double(i) = rational_bc(i).to_double(); // Convert rational to double
    }
    return bc_double;
}

// Rational barycentric to world conversion
Eigen::Matrix<wmtk::Rational, 3, 1> barycentricToWorldRational(
    const Eigen::Matrix<wmtk::Rational, 4, 1>& bc,
    const Eigen::Matrix<wmtk::Rational, 4, 3>& v)
{
    Eigen::Matrix<wmtk::Rational, 3, 1> result = Eigen::Matrix<wmtk::Rational, 3, 1>::Zero();
    for (int i = 0; i < 4; ++i) {
        result += bc(i) * v.row(i).transpose();
    }
    return result;
}
