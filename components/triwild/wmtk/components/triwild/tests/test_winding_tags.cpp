#include <wmtk/TriMesh.h>
#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/io/read_edge_mesh.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

// The per-input tags come from the winding number of that input's curve, which is a sum of
// SIGNED subtended angles. Anything that loses a curve's orientation -- reordering its
// segments, normalizing their endpoints -- leaves the loop closed and every structural
// check intact while turning the tags into noise, so the invariant worth testing is
// geometric, not structural: the area tagged for an input must equal the area its polygon
// encloses.
namespace {

// A uniform grid over [0,1]^2, split into triangles. Fine enough that the boundary
// triangles of the shapes below contribute a small fraction of their area.
void build_grid(TriWildMesh& mesh, int n)
{
    const int nv = (n + 1) * (n + 1);
    std::vector<std::array<size_t, 3>> tris;
    tris.reserve(2 * n * n);
    auto vid = [n](int i, int j) { return static_cast<size_t>(j * (n + 1) + i); };
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)}});
            tris.push_back({{vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    mesh.init(nv, tris);
    mesh.m_vertex_attribute.resize(nv);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) {
            auto& va = mesh.m_vertex_attribute[vid(i, j)];
            va.m_posf = Vector2d(double(i) / n, double(j) / n);
            va.m_is_rounded = true;
            va.m_pos = to_rational(va.m_posf);
        }
    }
}

/// Closed rectangle [x0,x1]x[y0,y1] as (V, E). `ccw` picks the traversal direction; the
/// tags must not depend on it.
void rect(double x0, double y0, double x1, double y1, bool ccw, MatrixXd& V, MatrixXi& E)
{
    V.resize(4, 2);
    V << x0, y0, x1, y0, x1, y1, x0, y1;
    E.resize(4, 2);
    for (int i = 0; i < 4; ++i) {
        const int a = i, b = (i + 1) % 4;
        E.row(i) = ccw ? Eigen::Vector2i(a, b) : Eigen::Vector2i(b, a);
    }
}

double tagged_area(const TriWildMesh& mesh, int64_t tag)
{
    double area = 0;
    for (const auto& f : mesh.get_faces()) {
        const size_t fid = f.fid(mesh);
        if (mesh.m_face_attribute[fid].tags.count(tag) == 0) {
            continue;
        }
        const auto vs = mesh.oriented_tri_vids(f);
        const Vector2d& a = mesh.m_vertex_attribute[vs[0]].m_posf;
        const Vector2d& b = mesh.m_vertex_attribute[vs[1]].m_posf;
        const Vector2d& c = mesh.m_vertex_attribute[vs[2]].m_posf;
        area += 0.5 * std::abs((b - a).x() * (c - a).y() - (b - a).y() * (c - a).x());
    }
    return area;
}

} // namespace

TEST_CASE("winding-tags-match-input-area", "[triwild][winding]")
{
    // Three disjoint rectangles of known area, one of them wound clockwise.
    const double tol = 2e-2; // boundary triangles straddle the outline
    Parameters params;
    TriWildMesh mesh(params, params.eps, 0);
    build_grid(mesh, 60);

    std::vector<MatrixXd> Vs(3);
    std::vector<MatrixXi> Es(3);
    rect(0.10, 0.10, 0.40, 0.30, true, Vs[0], Es[0]); // area 0.06
    rect(0.55, 0.20, 0.95, 0.50, false, Vs[1], Es[1]); // area 0.12, clockwise
    rect(0.30, 0.60, 0.50, 0.90, true, Vs[2], Es[2]); // area 0.06
    const std::array<double, 3> expected = {0.06, 0.12, 0.06};

    mesh.compute_winding_numbers(Vs, Es);

    for (int64_t k = 0; k < 3; ++k) {
        const double got = tagged_area(mesh, k);
        INFO("input " << k << ": tagged area " << got << ", expected " << expected[k]);
        CHECK(std::abs(got - expected[k]) < tol);
    }

    // Disjoint inputs must not share faces.
    for (const auto& f : mesh.get_faces()) {
        CHECK(mesh.m_face_attribute[f.fid(mesh)].tags.size() <= 1);
    }
}

TEST_CASE("winding-tags-survive-vertex-merge", "[triwild][winding]")
{
    // Same check, but the curves go through the reader that merges duplicate vertices --
    // the step that used to normalize edge endpoints and destroy the orientation. The
    // rectangle is written with a duplicated corner so the merge actually runs.
    namespace fs = std::filesystem;
    const fs::path path = fs::temp_directory_path() / "wmtk_winding_tag_test.obj";
    {
        std::ofstream f(path);
        f << "v 0.2 0.2 0\nv 0.7 0.2 0\nv 0.7 0.7 0\nv 0.2 0.7 0\nv 0.2 0.7 0\n";
        f << "l 1 2\nl 2 3\nl 3 4\nl 4 5\nl 5 1\n";
    }

    MatrixXd V;
    MatrixXi E;
    wmtk::io::read_edge_mesh(path.string(), V, E, 1e-8);
    fs::remove(path);
    V = V.block(0, 0, V.rows(), 2).eval();

    Parameters params;
    TriWildMesh mesh(params, params.eps, 0);
    build_grid(mesh, 60);
    mesh.compute_winding_numbers({V}, {E});

    const double got = tagged_area(mesh, 0);
    INFO("tagged area " << got << ", expected 0.25");
    CHECK(std::abs(got - 0.25) < 2e-2);
}
