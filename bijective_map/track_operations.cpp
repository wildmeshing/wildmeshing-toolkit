#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "track_operations.hpp"
#include "track_operations_curve.hpp"
// Function to save a vector<query_curve> to a file
void save_query_curves(const std::vector<query_curve>& curves, const std::string& filename)
{
    std::ofstream ofs(filename, std::ios::binary);

    // Write number of curves
    size_t num_curves = curves.size();
    ofs.write(reinterpret_cast<const char*>(&num_curves), sizeof(num_curves));

    // Save each query_curve
    for (const auto& curve : curves) {
        // Write number of segments
        size_t num_segments = curve.segments.size();
        ofs.write(reinterpret_cast<const char*>(&num_segments), sizeof(num_segments));

        // Write each segment
        for (const auto& segment : curve.segments) {
            ofs.write(reinterpret_cast<const char*>(&segment.f_id), sizeof(segment.f_id));
            ofs.write(reinterpret_cast<const char*>(segment.bcs[0].data()), sizeof(segment.bcs[0]));
            ofs.write(reinterpret_cast<const char*>(segment.bcs[1].data()), sizeof(segment.bcs[1]));
            ofs.write(reinterpret_cast<const char*>(segment.fv_ids.data()), sizeof(segment.fv_ids));
        }

        // Write next_segment_ids
        size_t num_next_ids = curve.next_segment_ids.size();
        ofs.write(reinterpret_cast<const char*>(&num_next_ids), sizeof(num_next_ids));
        ofs.write(
            reinterpret_cast<const char*>(curve.next_segment_ids.data()),
            num_next_ids * sizeof(int));
    }

    ofs.close();
}

// Function to load a vector<query_curve> from a file
std::vector<query_curve> load_query_curves(const std::string& filename)
{
    std::ifstream ifs(filename, std::ios::binary);
    std::vector<query_curve> curves;

    // Read number of curves
    size_t num_curves;
    ifs.read(reinterpret_cast<char*>(&num_curves), sizeof(num_curves));
    curves.resize(num_curves);

    // Load each query_curve
    for (auto& curve : curves) {
        // Read number of segments
        size_t num_segments;
        ifs.read(reinterpret_cast<char*>(&num_segments), sizeof(num_segments));
        curve.segments.resize(num_segments);

        // Read each segment
        for (auto& segment : curve.segments) {
            ifs.read(reinterpret_cast<char*>(&segment.f_id), sizeof(segment.f_id));
            ifs.read(reinterpret_cast<char*>(segment.bcs[0].data()), sizeof(segment.bcs[0]));
            ifs.read(reinterpret_cast<char*>(segment.bcs[1].data()), sizeof(segment.bcs[1]));
            ifs.read(reinterpret_cast<char*>(segment.fv_ids.data()), sizeof(segment.fv_ids));
        }

        // Read next_segment_ids
        size_t num_next_ids;
        ifs.read(reinterpret_cast<char*>(&num_next_ids), sizeof(num_next_ids));
        curve.next_segment_ids.resize(num_next_ids);
        ifs.read(
            reinterpret_cast<char*>(curve.next_segment_ids.data()),
            num_next_ids * sizeof(int));
    }

    ifs.close();
    return curves;
}


Eigen::Vector3d ComputeBarycentricCoordinates2D(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    // Calculate oriented areas using cross product (2D)
    auto oriented_area =
        [](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) {
            return (p2.x() - p1.x()) * (p3.y() - p1.y()) - (p3.x() - p1.x()) * (p2.y() - p1.y());
        };

    // Total area of triangle ABC
    double area_total = oriented_area(a, b, c);

    // Barycentric coordinates are ratios of sub-triangle areas to total area
    double area_pbc = oriented_area(p, b, c); // Area opposite to vertex A
    double area_apc = oriented_area(a, p, c); // Area opposite to vertex B
    double area_abp = oriented_area(a, b, p); // Area opposite to vertex C

    // Barycentric coordinates
    double u = area_pbc / area_total; // Weight for vertex A
    double v = area_apc / area_total; // Weight for vertex B
    double w = area_abp / area_total; // Weight for vertex C

    return Eigen::Vector3d(u, v, w);
}

// Rational version of ComputeBarycentricCoordinates2D
Eigen::Vector3<wmtk::Rational> ComputeBarycentricCoordinates2D_r(
    const Eigen::Vector2<wmtk::Rational>& p,
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c)
{
    Eigen::Vector2<wmtk::Rational> v0 = b - a, v1 = c - a, v2 = p - a;
    wmtk::Rational d00 = v0.dot(v0);
    wmtk::Rational d01 = v0.dot(v1);
    wmtk::Rational d11 = v1.dot(v1);
    wmtk::Rational d20 = v2.dot(v0);
    wmtk::Rational d21 = v2.dot(v1);
    wmtk::Rational denom = d00 * d11 - d01 * d01;

    // Check for degenerate triangle (collinear points)
    if (denom == wmtk::Rational(0)) {
        throw std::runtime_error("Degenerate triangle: points are collinear");
    }

    wmtk::Rational v = (d11 * d20 - d01 * d21) / denom;
    wmtk::Rational w = (d00 * d21 - d01 * d20) / denom;
    wmtk::Rational u = wmtk::Rational(1) - v - w;

    wmtk::Rational sum = u + v + w;
    if (sum == wmtk::Rational(0)) {
        throw std::runtime_error("Invalid barycentric coordinates: sum is zero");
    }

    return Eigen::Vector3<wmtk::Rational>(u, v, w) / sum;
}


// Precomputed barycentric helper (2D)
// defined in header

std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d(
    const Eigen::MatrixX<wmtk::Rational>& V2,
    const Eigen::MatrixXi& F)
{
    std::vector<BarycentricPrecompute2D> cache;
    cache.resize(F.rows());
    for (int i = 0; i < F.rows(); ++i) {
        Eigen::Matrix<wmtk::Rational, 2, 1> A = V2.row(F(i, 0)).head<2>().transpose();
        Eigen::Matrix<wmtk::Rational, 2, 1> B = V2.row(F(i, 1)).head<2>().transpose();
        Eigen::Matrix<wmtk::Rational, 2, 1> C = V2.row(F(i, 2)).head<2>().transpose();

        Eigen::Matrix<wmtk::Rational, 2, 2> M;
        M.col(0) = B - A;
        M.col(1) = C - A;

        wmtk::Rational det = M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0);
        if (det == wmtk::Rational(0)) {
            throw std::runtime_error("Degenerate triangle in barycentric precompute (det==0)");
        }
        Eigen::Matrix<wmtk::Rational, 2, 2> Minv;
        Minv(0, 0) = M(1, 1) / det;
        Minv(0, 1) = -M(0, 1) / det;
        Minv(1, 0) = -M(1, 0) / det;
        Minv(1, 1) = M(0, 0) / det;

        cache[i].Minv = Minv;
        cache[i].a = A;
    }
    return cache;
}

Eigen::Matrix<wmtk::Rational, 3, 1> barycentric_from_cache(
    const BarycentricPrecompute2D& bc,
    const Eigen::Matrix<wmtk::Rational, 2, 1>& p)
{
    Eigen::Matrix<wmtk::Rational, 2, 1> v2 = p - bc.a;
    Eigen::Matrix<wmtk::Rational, 2, 1> x = bc.Minv * v2; // [v; w]
    wmtk::Rational v = x(0);
    wmtk::Rational w = x(1);
    wmtk::Rational u = wmtk::Rational(1) - v - w;
    return Eigen::Matrix<wmtk::Rational, 3, 1>(u, v, w);
}

std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d_from_double(
    const Eigen::MatrixXd& V2,
    const Eigen::MatrixXi& F)
{
    Eigen::MatrixX<wmtk::Rational> V2_r(V2.rows(), V2.cols());
    for (int i = 0; i < V2.rows(); ++i) {
        for (int j = 0; j < V2.cols(); ++j) V2_r(i, j) = wmtk::Rational(V2(i, j));
    }
    return build_barycentric_cache_2d(V2_r, F);
}
