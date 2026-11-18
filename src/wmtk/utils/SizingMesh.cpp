#include "SizingMesh.hpp"

#include <wmtk/utils/Logger.hpp>

#include <igl/AABB.h>

#include <Eigen/Dense>

#include <vector>

namespace wmtk {

struct SizingMesh::Impl
{
    Kind kind;
    Eigen::MatrixXd V; // #V by 2 (Planar) or 3
    Eigen::MatrixXi C; // #C by 3 (triangles) or 4 (tets)
    Eigen::VectorXd length;
    igl::AABB<Eigen::MatrixXd, 2> tree2;
    igl::AABB<Eigen::MatrixXd, 3> tree3;

    // The length interpolated linearly in cell `c` at `w`, the barycentric weights of corners
    // 1.. (corner 0 takes the rest).
    template <int N>
    double interpolate(int c, const Eigen::Matrix<double, N, 1>& w) const
    {
        double l = (1 - w.sum()) * length(C(c, 0));
        for (int k = 0; k < N; ++k) l += w(k) * length(C(c, k + 1));
        return l;
    }
};

SizingMesh::SizingMesh(const MeshData& mesh, Kind kind, const std::string& option)
    : m_impl(std::make_unique<Impl>())
{
    Impl& d = *m_impl;
    d.kind = kind;

    const auto field = mesh.vertex_attributes.find(field_name);
    if (field == mesh.vertex_attributes.end()) {
        log_and_throw_error("`{}`: the mesh has no vertex field '{}'", option, field_name);
    }
    const DataMatrix<double> length = field->second.doubles();
    if (length.cols() != 1 || length.rows() != mesh.V.rows()) {
        log_and_throw_error("`{}`: '{}' must have one value per vertex", option, field_name);
    }
    if (!length.allFinite() || !(length.array() > 0).all()) {
        log_and_throw_error("`{}`: every '{}' must be positive", option, field_name);
    }

    const CellType type = kind == Kind::Volume ? CellType::Tetrahedron : CellType::Triangle;
    const auto cells = mesh.cells.find(type);
    if (cells == mesh.cells.end() || cells->second.connectivity.rows() == 0) {
        log_and_throw_error(
            "`{}`: the mesh has no {}",
            option,
            kind == Kind::Volume ? "tetrahedra" : "triangles");
    }
    d.C = cells->second.connectivity.cast<int>();
    if (d.C.minCoeff() < 0 || d.C.maxCoeff() >= mesh.V.rows()) {
        log_and_throw_error("`{}`: a cell refers to a missing vertex", option);
    }
    d.length = length.col(0);

    if (kind == Kind::Planar) {
        d.V = mesh.V.leftCols(2);
        d.tree2.init(d.V, d.C);
    } else {
        d.V = mesh.V;
        d.tree3.init(d.V, d.C);
    }
    logger().info(
        "{}: {} {}, {} from {} to {}",
        option,
        d.C.rows(),
        kind == Kind::Volume ? "tets" : "triangles",
        field_name,
        d.length.minCoeff(),
        d.length.maxCoeff());
}

SizingMesh::~SizingMesh() = default;

SizingMesh::Kind SizingMesh::kind() const
{
    return m_impl->kind;
}

double SizingMesh::min_length() const
{
    return m_impl->length.minCoeff();
}

double SizingMesh::max_length() const
{
    return m_impl->length.maxCoeff();
}

double SizingMesh::operator()(const Eigen::Vector2d& p) const
{
    const Impl& d = *m_impl;
    assert(d.kind == Kind::Planar);
    // Dynamic size: libigl's AABB::find compiles its 3D branch for DIM 2 too, which a fixed
    // 2-vector does not fit.
    const Eigen::RowVectorXd q = p.transpose();
    const std::vector<int> hits = d.tree2.find(d.V, d.C, q, true);
    if (hits.empty()) return -1;
    const int c = hits[0];
    const Eigen::Vector2d a = d.V.row(d.C(c, 0)), b = d.V.row(d.C(c, 1)), e = d.V.row(d.C(c, 2));
    Eigen::Matrix2d M;
    M << b - a, e - a;
    const double det = M.determinant();
    if (det == 0) return d.length(d.C(c, 0));
    const Eigen::Vector2d w = M.inverse() * (p - a);
    return d.interpolate<2>(c, w);
}

double SizingMesh::operator()(const Eigen::Vector3d& p) const
{
    const Impl& d = *m_impl;
    if (d.kind == Kind::Volume) {
        const Eigen::RowVector3d q = p.transpose();
        const std::vector<int> hits = d.tree3.find(d.V, d.C, q, true);
        if (hits.empty()) return -1;
        const int c = hits[0];
        const Eigen::Vector3d a = d.V.row(d.C(c, 0));
        Eigen::Matrix3d M;
        for (int k = 0; k < 3; ++k) M.col(k) = d.V.row(d.C(c, k + 1)).transpose() - a;
        if (M.determinant() == 0) return d.length(d.C(c, 0));
        const Eigen::Vector3d w = M.partialPivLu().solve(p - a);
        return d.interpolate<3>(c, w);
    }
    assert(d.kind == Kind::Surface);
    // The closest point of the surface, and its barycentric weights in its triangle.
    int c = -1;
    Eigen::RowVector3d closest;
    d.tree3.squared_distance(d.V, d.C, Eigen::RowVector3d(p.transpose()), c, closest);
    if (c < 0) return -1;
    const Eigen::Vector3d a = d.V.row(d.C(c, 0));
    const Eigen::Vector3d u = d.V.row(d.C(c, 1)).transpose() - a;
    const Eigen::Vector3d v = d.V.row(d.C(c, 2)).transpose() - a;
    const Eigen::Vector3d r = closest.transpose() - a;
    const double uu = u.dot(u), uv = u.dot(v), vv = v.dot(v);
    const double det = uu * vv - uv * uv;
    if (det == 0) return d.length(d.C(c, 0));
    const double ru = r.dot(u), rv = r.dot(v);
    const Eigen::Vector2d w((vv * ru - uv * rv) / det, (uu * rv - uv * ru) / det);
    return d.interpolate<2>(c, w);
}

} // namespace wmtk
