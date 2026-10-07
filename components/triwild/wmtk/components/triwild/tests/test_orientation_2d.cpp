// The 2D counterpart of tetwild's test_orientation.cpp: the tracked curves carry the input's
// direction (SurfaceTagAttributes::m_orientation) from embed_segments through every
// operation, so filter "tracked" agrees with filter "input" on the configurations a guess gets
// wrong -- several loops, one inside out, one nested, two sharing an edge, one resting on
// another.

#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/utils/EmbedSegments.hpp>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <map>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

/// Closed polylines, welding exactly coincident vertices so loops can share an edge.
struct Loops
{
    std::vector<Vector2d> v;
    std::vector<std::array<int, 2>> e;
    std::map<std::array<double, 2>, int> index;

    int vertex(const Vector2d& p)
    {
        const std::array<double, 2> key{{p[0], p[1]}};
        const auto it = index.find(key);
        if (it != index.end()) return it->second;
        index.emplace(key, int(v.size()));
        v.push_back(p);
        return int(v.size()) - 1;
    }

    /// Axis-aligned square, counterclockwise (winding number +1 inside) unless `clockwise`.
    void square(const Vector2d& lo, const Vector2d& hi, bool clockwise = false)
    {
        const std::array<int, 4> c{
            {vertex(lo), vertex({hi[0], lo[1]}), vertex(hi), vertex({lo[0], hi[1]})}};
        for (int i = 0; i < 4; ++i) {
            const int a = c[i], b = c[(i + 1) % 4];
            if (clockwise) {
                e.push_back({{b, a}});
            } else {
                e.push_back({{a, b}});
            }
        }
    }

    MatrixXd V() const
    {
        MatrixXd m(v.size(), 2);
        for (size_t i = 0; i < v.size(); ++i) m.row(i) = v[i];
        return m;
    }
    MatrixXi E() const
    {
        MatrixXi m(e.size(), 2);
        for (size_t i = 0; i < e.size(); ++i) m.row(i) << e[i][0], e[i][1];
        return m;
    }
};

double face_area(TriWildMesh& mesh, size_t fid)
{
    const auto vs = mesh.oriented_tri_vids(fid);
    const auto& p = mesh.m_vertex_attribute;
    const Vector2d a = p[vs[1]].m_posf - p[vs[0]].m_posf, b = p[vs[2]].m_posf - p[vs[0]].m_posf;
    return 0.5 * (a[0] * b[1] - a[1] * b[0]);
}

/// Area inside by the input winding number (tags) and by the tracked one, recomputing both.
std::array<double, 2> areas(TriWildMesh& mesh, const MatrixXd& V, const MatrixXi& E)
{
    for (const auto& f : mesh.get_faces()) mesh.m_face_attribute[f.fid(mesh)].tags.clear();
    mesh.compute_winding_numbers({V}, {E});
    mesh.compute_tracked_winding_number();
    std::array<double, 2> res{{0, 0}};
    for (const auto& f : mesh.get_faces()) {
        const size_t fid = f.fid(mesh);
        const double a = std::abs(face_area(mesh, fid));
        if (!mesh.m_face_attribute[fid].tags.empty()) res[0] += a;
        if (mesh.m_face_attribute[fid].m_winding_number > 0.5) res[1] += a;
    }
    return res;
}

void check_case(const char* name, const Loops& l, double expected)
{
    const MatrixXd V = l.V();
    const MatrixXi E = l.E();
    Parameters params;
    params.init(V.colwise().minCoeff(), V.colwise().maxCoeff());

    MatrixXd Vo;
    std::vector<Vector2r> Vr;
    MatrixXi F, Eo;
    std::vector<int> orientation;
    utils::embed_segments(V, E, Vo, Vr, F, Eo, nullptr, &orientation);

    TriWildMesh mesh(params, params.eps, 0);
    mesh.init_mesh(Vo, Vr, F, Eo, {"input"}, V, E, &orientation);
    REQUIRE(mesh.m_tracks_orientation);

    // Right after the insertion the tracked curves ARE the input: closed, and the two winding
    // numbers agree face by face (no face of a 2D arrangement is flat).
    CHECK(mesh.tracked_curve_boundary().empty());
    const auto a0 = areas(mesh, V, E);
    size_t disagree = 0;
    for (const auto& f : mesh.get_faces()) {
        const auto& attr = mesh.m_face_attribute[f.fid(mesh)];
        if (attr.tags.empty() == (attr.m_winding_number > 0.5)) ++disagree;
    }
    logger().info(
        "[orientation-2d] {}: {} disagree after insertion, areas {} {}",
        name,
        disagree,
        a0[0],
        a0[1]);
    CHECK(disagree == 0);
    CHECK(std::abs(a0[1] - expected) < 1e-9 * expected);

    mesh.mesh_improvement(3);
    CHECK(mesh.tracked_curve_boundary().empty());
    const auto a1 = areas(mesh, V, E);
    logger().info(
        "[orientation-2d] {}: after optimization, input {} tracked {} expected {}",
        name,
        a1[0],
        a1[1],
        expected);
    CHECK(std::abs(a1[1] - expected) < 0.02 * expected);
    CHECK(std::abs(a1[0] - expected) < 0.02 * expected);
}

} // namespace

TEST_CASE("orientation-2d-single-square", "[triwild][orientation]")
{
    Loops l;
    l.square({0, 0}, {1, 1});
    check_case("single square", l, 1);
}

TEST_CASE("orientation-2d-clockwise-square", "[triwild][orientation]")
{
    // Alone and clockwise: both winding numbers take the same whole-input flip.
    Loops l;
    l.square({0, 0}, {1, 1}, true);
    check_case("clockwise square", l, 1);
}

TEST_CASE("orientation-2d-square-and-clockwise-square", "[triwild][orientation]")
{
    Loops l;
    l.square({0, 0}, {1, 1});
    l.square({2, 0}, {3, 1}, true);
    check_case("square + clockwise square", l, 1);
}

TEST_CASE("orientation-2d-nested-squares", "[triwild][orientation]")
{
    Loops l;
    l.square({0, 0}, {3, 3});
    l.square({1, 1}, {2, 2});
    check_case("nested squares", l, 9);
}

TEST_CASE("orientation-2d-squares-sharing-an-edge", "[triwild][orientation]")
{
    // The shared edge is traversed once each way and cancels.
    Loops l;
    l.square({0, 0}, {1, 1});
    l.square({1, 0}, {2, 1});
    check_case("squares sharing an edge", l, 2);
}

TEST_CASE("orientation-2d-square-resting-on-a-strip", "[triwild][orientation]")
{
    // The upper square's bottom lies inside the strip's top edge without sharing its vertices.
    Loops l;
    l.square({0, 0}, {3, 1});
    l.square({1, 1}, {2, 2});
    check_case("square resting on a strip", l, 4);
}
