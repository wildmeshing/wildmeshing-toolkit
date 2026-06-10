#include "Delaunay.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <Delaunay_psm.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/predicates/predicates.h>
#include <wmtk/utils/Logger.hpp>


#include <cassert>
#include <mutex>

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

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
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

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return true;
    return false;
}
} // namespace

namespace wmtk::delaunay {

auto delaunay3D(const std::vector<Point3D>& points)
    -> std::pair<std::vector<Point3D>, std::vector<Tetrahedron>>
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(3, "BDEL");
    assert(engine);

    // Some settings
    engine->set_reorder(true); // making this true fixed a bug for whatever reason
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(points.size() > 0);
    engine->set_vertices((GEO::index_t)points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_tets = engine->nb_cells();
    std::vector<Point3D> vertices(num_vertices);
    std::vector<Tetrahedron> tets(num_tets);

    for (GEO::index_t i = 0; i < num_vertices; i++) {
        for (size_t j = 0; j < 3; j++) {
            vertices[i][j] = engine->vertex_ptr(i)[j];
        }
    }

    for (GEO::index_t i = 0; i < num_tets; i++) {
        for (GEO::index_t j = 0; j < 4; j++) {
            tets[i][j] = engine->cell_vertex(i, j);
        }
    }

    // sort tets
    for (auto& tri : tets) {
        std::sort(tri.begin(), tri.end());
    }
    std::sort(tets.begin(), tets.end());

    for (auto& tri : tets) {
        const Point3D p0 = points[tri[0]];
        const Point3D p1 = points[tri[1]];
        const Point3D p2 = points[tri[2]];
        const Point3D p3 = points[tri[3]];
        if (is_inverted(p0, p1, p2, p3)) {
            // std::cout << "Inverted tet found" << std::endl;
            std::swap(tri[2], tri[3]); // invert tet
        }
    }

    return {vertices, tets};
}

auto delaunay2D(const std::vector<Point2D>& points)
    -> std::pair<std::vector<Point2D>, std::vector<Triangle>>
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []() { GEO::initialize(); });

    GEO::Delaunay_var engine = GEO::Delaunay::create(2, "BDEL2d");
    assert(engine);

    // Some settings
    engine->set_reorder(true);
    engine->set_stores_cicl(false); // Incident tetrahedral list.
    engine->set_stores_neighbors(false); // Vertex neighbors.
    engine->set_refine(false);
    engine->set_keeps_infinite(false);

    // Run!
    geo_assert(points.size() > 0);
    engine->set_vertices((GEO::index_t)points.size(), points.front().data());

    // Extract output.
    const size_t num_vertices = engine->nb_vertices();
    const size_t num_triangles = engine->nb_cells();
    std::vector<Point2D> vertices(num_vertices);
    std::vector<Triangle> triangles(num_triangles);

    for (GEO::index_t i = 0; i < num_vertices; i++) {
        for (GEO::index_t j = 0; j < 2; j++) {
            vertices[i][j] = engine->vertex_ptr(i)[j];
        }
    }

    for (GEO::index_t i = 0; i < num_triangles; i++) {
        for (GEO::index_t j = 0; j < 3; j++) {
            triangles[i][j] = engine->cell_vertex(i, j);
        }
    }

    // sort tets
    for (auto& tri : triangles) {
        std::sort(tri.begin(), tri.end());
    }
    std::sort(triangles.begin(), triangles.end());

    for (auto& tri : triangles) {
        const Point2D p0 = points[tri[0]];
        const Point2D p1 = points[tri[1]];
        const Point2D p2 = points[tri[2]];
        if (is_inverted(p0, p1, p2)) {
            // std::cout << "Inverted tet found" << std::endl;
            std::swap(tri[1], tri[2]); // invert triangle
        }
    }

    return {vertices, triangles};
}

void constrained_delaunay2D(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out)
{
    if (V.cols() != 2) {
        log_and_throw_error("Input vertices must be 2D");
    }
    if (E.cols() != 2) {
        log_and_throw_error("Input edges must be 2D");
    }

    GEO::CDT2d cdt;
    // GEO::ExactCDT2d cdt;
    std::vector<GEO::index_t> vertex_ids(V.rows());

    cdt.create_enclosing_rectangle(
        V.col(0).minCoeff(),
        V.col(1).minCoeff(),
        V.col(0).maxCoeff(),
        V.col(1).maxCoeff());

    for (size_t i = 0; i < V.rows(); ++i) {
        const GEO::vec2 p(V(i, 0), V(i, 1));
        const GEO::index_t id = cdt.insert(p);
        vertex_ids[i] = id;
    }
    for (size_t i = 0; i < E.rows(); ++i) {
        const int v0 = E(i, 0);
        const int v1 = E(i, 1);
        if (v0 >= V.rows() || v1 >= V.rows()) {
            log_and_throw_error("Edge index out of bounds at index {}: {}", i, E.row(i));
        }
        cdt.insert_constraint(vertex_ids[v0], vertex_ids[v1]);
    }

    GEO::index_t nv = cdt.nv();
    GEO::index_t nt = cdt.nT();

    // get constrained edges
    std::set<std::pair<int, int>> constrained_edges;

    for (GEO::index_t t = 0; t < nt; ++t) {
        for (GEO::index_t le = 0; le < 3; ++le) {
            if (cdt.Tedge_cnstr_first(t, le) == GEO::NO_INDEX) {
                continue;
            }

            int a = int(cdt.Tv(t, (le + 1) % 3));
            int b = int(cdt.Tv(t, (le + 2) % 3));

            if (a > b) {
                std::swap(a, b);
            }
            constrained_edges.insert({a, b});
        }
    }

    logger().info("CDT: #V = {}, #F = {}, #E_constrained = {}", nv, nt, constrained_edges.size());

    V_out.resize(nv, 2);

    for (GEO::index_t v = 0; v < nv; ++v) {
        const GEO::vec2& p = cdt.point(v);
        V_out(v, 0) = p.x;
        V_out(v, 1) = p.y;
    }

    F_out.resize(nt, 3);

    for (GEO::index_t t = 0; t < nt; ++t) {
        F_out(t, 0) = int(cdt.Tv(t, 0));
        F_out(t, 1) = int(cdt.Tv(t, 1));
        F_out(t, 2) = int(cdt.Tv(t, 2));
    }

    E_out.resize(constrained_edges.size(), 2);
    {
        int idx = 0;
        for (const auto& edge : constrained_edges) {
            E_out(idx, 0) = edge.first;
            E_out(idx, 1) = edge.second;
            ++idx;
        }
    }
}

} // namespace wmtk::delaunay
