#pragma once

#include <Eigen/Core>
#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <wmtk/Types.hpp>

namespace wmtk::components::prismatic_mesh::internal {

// A frozen, piecewise-affine chart of the current offset triangle patch.
// Only its two selected vertices move; the other chart vertices stay fixed.
class SurfaceDomain
{
public:
    using UV = Eigen::Vector2d;
    using Triangle = std::array<size_t, 3>;

    static std::optional<SurfaceDomain> build(
        const MatrixXd& positions,
        const std::vector<Triangle>& faces,
        const std::array<size_t, 2>& edge,
        std::string& failure);

    UV coordinate(size_t vertex) const;
    std::optional<Vector3d> position(const UV& uv) const;
    // Select the best one-sided gradient at chart seams and original vertices.
    UV descent(const UV& uv, const Vector3d& gradient) const;
    double step_limit(const std::array<UV, 2>& direction) const;
    bool valid(const std::array<UV, 2>& points) const;
    size_t triangle_count() const { return triangles.size(); }
    size_t vertex_count() const { return vertices.size(); }

private:
    struct ChartTriangle
    {
        Triangle vertices;
        Eigen::Matrix2d inverse;
        Eigen::Matrix<double, 3, 2> jacobian;
    };
    std::vector<size_t> vertices, boundary;
    std::map<size_t, size_t> local;
    MatrixXd points, parameters;
    std::array<size_t, 2> ends;
    std::vector<ChartTriangle> triangles;

    Eigen::Vector3d barycentric(const ChartTriangle& tri, const UV& uv) const;
};
} // namespace wmtk::components::prismatic_mesh::internal
