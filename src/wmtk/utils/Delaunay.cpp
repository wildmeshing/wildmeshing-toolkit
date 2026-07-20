#include "Delaunay.hpp"

// clang-format off
#include <VolumeRemesher/delaunay.h>
#include <VolumeRemesher/2d/delaunay2d.h>
// clang-format on

#include <igl/predicates/predicates.h>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <vector>

namespace {
bool is_inverted(
    const wmtk::delaunay::Point3D& p0,
    const wmtk::delaunay::Point3D& p1,
    const wmtk::delaunay::Point3D& p2,
    const wmtk::delaunay::Point3D& p3)
{
    const Eigen::Vector3d v0(p0[0], p0[1], p0[2]);
    const Eigen::Vector3d v1(p1[0], p1[1], p1[2]);
    const Eigen::Vector3d v2(p2[0], p2[1], p2[2]);
    const Eigen::Vector3d v3(p3[0], p3[1], p3[2]);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(v0, v1, v2, v3);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // neg result == pos tet
        return false;
    return true;
}

bool is_inverted(
    const wmtk::delaunay::Point2D& p0,
    const wmtk::delaunay::Point2D& p1,
    const wmtk::delaunay::Point2D& p2)
{
    const Eigen::Vector2d v0(p0[0], p0[1]);
    const Eigen::Vector2d v1(p1[0], p1[1]);
    const Eigen::Vector2d v2(p2[0], p2[1]);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(v0, v1, v2);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) return true;
    return false;
}

/**
 * @brief Collapse coincident input points, keeping one representative each.
 *
 * Both VolumeRemesher kernels require duplicate-free input -- the 3D one states the
 * assumption outright and would otherwise walk into ip_error(), which calls exit() --
 * whereas geogram used to absorb duplicates internally. So deduplicate here.
 *
 * Comparing doubles exactly is complete for this: two different doubles are never the same
 * point, and equal doubles always are. The index tie-break makes the comparator a strict
 * total order, so which duplicate survives does not depend on sort stability.
 *
 * @param[in] points the caller's points
 * @param[out] flat DIM * n_unique interleaved coordinates, the kernels' input layout
 * @param[out] to_original for each unique point, the caller index it came from -- so the
 *             cells we hand back still index the caller's array, duplicates and all
 */
template <size_t DIM>
void unique_points(
    const std::vector<std::array<double, DIM>>& points,
    std::vector<double>& flat,
    std::vector<size_t>& to_original)
{
    std::vector<size_t> order(points.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&points](size_t a, size_t b) {
        if (points[a] != points[b]) return points[a] < points[b];
        return a < b;
    });

    flat.clear();
    to_original.clear();
    flat.reserve(DIM * points.size());
    to_original.reserve(points.size());
    for (size_t k = 0; k < order.size(); ++k) {
        if (k > 0 && points[order[k]] == points[order[k - 1]]) continue;
        to_original.push_back(order[k]);
        for (size_t c = 0; c < DIM; ++c) flat.push_back(points[order[k]][c]);
    }
}

/**
 * @brief Whether four of the points span a volume.
 *
 * Mirrors the search in vol_rem::TetMesh::init(): fix the first two points and look for a
 * third and fourth that are not coplanar with them. init() calls ip_error() when it fails,
 * which exits the process, so the check has to happen before we hand anything over. The
 * geogram-backed implementation returned an empty tetrahedrization for such input.
 */
bool spans_a_volume(const std::vector<double>& coords, size_t n)
{
    if (n < 4) return false;

    const auto pt = [&coords](size_t i) {
        return Eigen::Vector3d(coords[3 * i], coords[3 * i + 1], coords[3 * i + 2]);
    };

    igl::predicates::exactinit();
    const Eigen::Vector3d p0 = pt(0);
    const Eigen::Vector3d p1 = pt(1);
    for (size_t k = 2; k + 1 < n; ++k) {
        for (size_t l = k + 1; l < n; ++l) {
            if (igl::predicates::orient3d(p0, p1, pt(k), pt(l)) !=
                igl::predicates::Orientation::COPLANAR) {
                return true;
            }
        }
    }
    return false;
}
} // namespace

namespace wmtk::delaunay {

auto delaunay3D(const std::vector<Point3D>& points)
    -> std::pair<std::vector<Point3D>, std::vector<Tetrahedron>>
{
    std::vector<double> coords;
    std::vector<size_t> to_original;
    unique_points(points, coords, to_original);
    const size_t n = to_original.size();

    std::vector<Tetrahedron> tets;

    // Degenerate input (fewer than four distinct points, or all of them coplanar) has no
    // tetrahedrization; report it as an empty one rather than letting the kernel exit().
    if (spans_a_volume(coords, n)) {
        vol_rem::TetMesh mesh;

        // ~TetMesh frees this with free(), so it has to come from malloc.
        mesh.vertices = static_cast<vol_rem::vertex_t*>(std::malloc(n * sizeof(vol_rem::vertex_t)));
        if (mesh.vertices == nullptr) {
            log_and_throw_error("delaunay3D: could not allocate {} vertices", n);
        }
        mesh.num_vertices = static_cast<uint32_t>(n);
        for (size_t i = 0; i < n; ++i) {
            mesh.vertices[i].coord[0] = coords[3 * i + 0];
            mesh.vertices[i].coord[1] = coords[3 * i + 1];
            mesh.vertices[i].coord[2] = coords[3 * i + 2];
            // tetrahedrize() permutes the vertex array; this is what maps a tet node back.
            mesh.vertices[i].original_index = static_cast<uint32_t>(i);
        }

        mesh.tetrahedrize();

        tets.reserve(mesh.tet_num);
        for (uint64_t t = 0; t < mesh.tet_num; ++t) {
            const uint32_t* tn = mesh.tet_node + 4 * t;
            // Ghost tets close the convex hull off against the vertex at infinity; they are
            // not part of the triangulation.
            if (tn[0] == INFINITE_VERTEX || tn[1] == INFINITE_VERTEX || tn[2] == INFINITE_VERTEX ||
                tn[3] == INFINITE_VERTEX) {
                continue;
            }
            Tetrahedron tet;
            for (size_t j = 0; j < 4; ++j) {
                tet[j] = to_original[mesh.vertices[tn[j]].original_index];
            }
            tets.push_back(tet);
        }
    }

    // sort tets
    for (auto& tet : tets) {
        std::sort(tet.begin(), tet.end());
    }
    std::sort(tets.begin(), tets.end());

    for (auto& tet : tets) {
        const Point3D p0 = points[tet[0]];
        const Point3D p1 = points[tet[1]];
        const Point3D p2 = points[tet[2]];
        const Point3D p3 = points[tet[3]];
        if (is_inverted(p0, p1, p2, p3)) {
            std::swap(tet[2], tet[3]); // invert tet
        }
    }

    return {points, tets};
}

auto delaunay2D(const std::vector<Point2D>& points)
    -> std::pair<std::vector<Point2D>, std::vector<Triangle>>
{
    std::vector<double> coords;
    std::vector<size_t> to_original;
    unique_points(points, coords, to_original);
    const size_t n = to_original.size();

    std::vector<Triangle> triangles;

    // set_vertices does not copy the coordinates, so `coords` has to outlive the walk below.
    // It returns false when the points are all collinear, which -- together with the n < 3
    // guard -- is the degenerate input the geogram-backed implementation reported as an empty
    // triangulation.
    vol_rem::vr2d::Delaunay2d del;
    if (n >= 3 && del.set_vertices(static_cast<vol_rem::vr2d::index_t>(n), coords.data())) {
        // The finite triangles are ordered first; the rest are virtual ones covering the
        // region outside the convex hull.
        const auto num_finite = del.nb_finite_triangles();
        triangles.reserve(num_finite);
        for (vol_rem::vr2d::index_t t = 0; t < num_finite; ++t) {
            Triangle tri;
            for (size_t j = 0; j < 3; ++j) {
                tri[j] =
                    to_original[del.triangle_vertex(t, static_cast<vol_rem::vr2d::index_t>(j))];
            }
            triangles.push_back(tri);
        }
    }

    // sort triangles
    for (auto& tri : triangles) {
        std::sort(tri.begin(), tri.end());
    }
    std::sort(triangles.begin(), triangles.end());

    for (auto& tri : triangles) {
        const Point2D p0 = points[tri[0]];
        const Point2D p1 = points[tri[1]];
        const Point2D p2 = points[tri[2]];
        if (is_inverted(p0, p1, p2)) {
            std::swap(tri[1], tri[2]); // invert triangle
        }
    }

    return {points, triangles};
}

} // namespace wmtk::delaunay
