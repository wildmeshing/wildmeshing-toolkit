// The tracked surface carries the input's orientation (SurfaceTagAttributes::m_orientation)
// from the insertion through every operation, so the tracked-surface winding number is the
// input's own rather than a per-patch guess. These cases are the configurations a guess gets
// wrong: several components, one of them inside out, one nested in another, two touching
// face-to-face, one resting on another.

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/components/tetwild/Parameters.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <map>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

/// A triangle soup that welds exactly coincident vertices, so two boxes sharing a face share
/// its vertices -- which is what puts their opposite triangles into one coplanar group.
struct Soup
{
    std::vector<Vector3d> v;
    std::vector<std::array<size_t, 3>> f;
    std::map<std::array<double, 3>, size_t> index;

    size_t vertex(const Vector3d& p)
    {
        const std::array<double, 3> key{{p[0], p[1], p[2]}};
        const auto it = index.find(key);
        if (it != index.end()) return it->second;
        index.emplace(key, v.size());
        v.push_back(p);
        return v.size() - 1;
    }

    /// Axis-aligned box, outward-facing unless `inward`.
    void box(const Vector3d& lo, const Vector3d& hi, bool inward = false)
    {
        std::array<size_t, 8> id;
        for (int i = 0; i < 8; ++i) {
            id[i] = vertex(
                Vector3d(i & 1 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 4 ? hi[2] : lo[2]));
        }
        // Outward for vertex i = x + 2y + 4z.
        const int tris[12][3] = {
            {0, 2, 3},
            {0, 3, 1},
            {4, 5, 7},
            {4, 7, 6},
            {0, 1, 5},
            {0, 5, 4},
            {2, 6, 7},
            {2, 7, 3},
            {0, 4, 6},
            {0, 6, 2},
            {1, 3, 7},
            {1, 7, 5}};
        for (const auto& t : tris) {
            if (inward) {
                f.push_back({{id[t[0]], id[t[2]], id[t[1]]}});
            } else {
                f.push_back({{id[t[0]], id[t[1]], id[t[2]]}});
            }
        }
    }
};

double volume_where(TetWildMesh& mesh, bool tracked)
{
    double vol = 0;
    for (const auto& t : mesh.get_tets()) {
        const size_t tid = t.tid(mesh);
        const auto& a = mesh.tet_finalize(tid);
        if ((tracked ? a.m_winding_number_tracked : a.m_winding_number_input) <= 0.5) continue;
        const auto vs = mesh.oriented_tet_vids(tid);
        const auto& p = mesh.m_vertex_attribute;
        vol += std::abs((p[vs[1]].m_posf - p[vs[0]].m_posf)
                            .cross(p[vs[2]].m_posf - p[vs[0]].m_posf)
                            .dot(p[vs[3]].m_posf - p[vs[0]].m_posf)) /
               6;
    }
    return vol;
}

void compute_both_winding_numbers(TetWildMesh& mesh, const Soup& s)
{
    const auto tets = mesh.get_tets();
    const Eigen::MatrixXd c = mesh.tet_barycenters(tets);
    mesh.compute_winding_number(tets, c, s.v, s.f);
    mesh.compute_winding_number(tets, c);
}

/// Insert, check, optimize a little, check again. `expected` is the volume where the input's
/// winding number exceeds 1/2.
void check_case(const char* name, const Soup& s, double expected)
{
    Parameters params;
    params.init(s.v, s.f);

    components::shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(s.v, 0);
    surf_mesh.create_mesh(s.v.size(), s.f, {}, params.eps);
    const std::shared_ptr<SampleEnvelope> env(&surf_mesh.m_envelope, [](SampleEnvelope*) {});

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<int> tet_face_orientation;
    {
        TetWildMesh mesh_insertion(params, env, 0);
        mesh_insertion.insertion_by_volumeremesher(
            s.v,
            s.f,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation);
    }
    TetWildMesh mesh(params, env, 0);
    mesh.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        &tet_face_orientation);
    REQUIRE(mesh.m_tracks_orientation);

    // Right after the insertion the tracked surface IS the input, so the two winding numbers
    // agree tet by tet, and the oriented surface is closed. Except on the arrangement's flat
    // tets: lying in the surface, with their barycenter on it, they have no winding number to
    // agree on (the evaluation lands anywhere around 1/2), and the optimizer removes them.
    CHECK(mesh.tracked_surface_boundary().empty());
    compute_both_winding_numbers(mesh, s);
    size_t disagree = 0, flat = 0;
    for (const auto& t : mesh.get_tets()) {
        const size_t tid = t.tid(mesh);
        const auto vs = mesh.oriented_tet_vids(tid);
        const auto& p = mesh.m_vertex_attribute;
        const double vol = (p[vs[1]].m_posf - p[vs[0]].m_posf)
                               .cross(p[vs[2]].m_posf - p[vs[0]].m_posf)
                               .dot(p[vs[3]].m_posf - p[vs[0]].m_posf);
        if (std::abs(vol) < 1e-12) {
            ++flat;
            continue;
        }
        const auto& a = mesh.tet_finalize(tid);
        if (std::lround(a.m_winding_number_input) != std::lround(a.m_winding_number_tracked)) {
            ++disagree;
        }
    }
    logger().info(
        "[orientation] {}: {} tets ({} flat), {} disagree after insertion",
        name,
        tets.size(),
        flat,
        disagree);
    CHECK(disagree == 0);
    CHECK(std::abs(volume_where(mesh, true) - expected) < 1e-6 * expected);

    // The operations must carry the orientation: still closed, and the tracked surface -- now
    // within eps of the input rather than on it -- still encloses the same solid.
    mesh.mesh_improvement(3);
    CHECK(mesh.tracked_surface_boundary().empty());
    compute_both_winding_numbers(mesh, s);
    const double vt = volume_where(mesh, true), vi = volume_where(mesh, false);
    logger().info(
        "[orientation] {}: after optimization, tracked {} input {} expected {}",
        name,
        vt,
        vi,
        expected);
    CHECK(std::abs(vt - expected) < 0.02 * expected);
    CHECK(std::abs(vi - expected) < 0.02 * expected);
}

} // namespace

TEST_CASE("orientation-single-box", "[tetwild][orientation]")
{
    Soup s;
    s.box({0, 0, 0}, {1, 1, 1});
    check_case("single box", s, 1);
}

TEST_CASE("orientation-inside-out-box", "[tetwild][orientation]")
{
    // Alone, inside out: both winding numbers take the same whole-surface flip.
    Soup s;
    s.box({0, 0, 0}, {1, 1, 1}, true);
    check_case("inside-out box", s, 1);
}

TEST_CASE("orientation-box-and-inside-out-box", "[tetwild][orientation]")
{
    // The inside-out one is a negative solid and must stay out; the other one in.
    Soup s;
    s.box({0, 0, 0}, {1, 1, 1});
    s.box({2, 0, 0}, {3, 1, 1}, true);
    check_case("box + inside-out box", s, 1);
}

TEST_CASE("orientation-nested-boxes", "[tetwild][orientation]")
{
    // Winding number 2 inside the inner box: still inside.
    Soup s;
    s.box({0, 0, 0}, {3, 3, 3});
    s.box({1, 1, 1}, {2, 2, 2});
    check_case("nested boxes", s, 27);
}

TEST_CASE("orientation-boxes-sharing-a-face", "[tetwild][orientation]")
{
    // The shared face carries two opposite, coincident triangle pairs on the same vertices: one
    // coplanar group with both orientations, which cancel to 0 there.
    Soup s;
    s.box({0, 0, 0}, {1, 1, 1});
    s.box({1, 0, 0}, {2, 1, 1});
    check_case("boxes sharing a face", s, 2);
}

TEST_CASE("orientation-box-resting-on-a-plate", "[tetwild][orientation]")
{
    // Thingi10K 52564's configuration: the peg's bottom lies inside the plate's top without
    // sharing vertices, so the facets under it belong to two coplanar groups of opposite
    // orientation.
    Soup s;
    s.box({0, 0, 0}, {3, 3, 1});
    s.box({1, 1, 1}, {2, 2, 2});
    check_case("box resting on a plate", s, 10);
}

TEST_CASE("orientation-box-resting-along-a-plate-edge", "[tetwild][orientation]")
{
    // The box's bottom shares an edge with the plate's top, so the two opposite faces are one
    // coplanar group of mixed orientation: the exact per-point path. (Thingi10K 104513 also has
    // a facet straddling an uncut edge between two of such a group's triangles; this
    // arrangement happens to cut along the plate's diagonal, so it does not reproduce that.)
    Soup s;
    s.box({0, 0, 0}, {3, 3, 1});
    s.box({0, 0, 1}, {3, 1, 2});
    check_case("box resting along a plate edge", s, 12);
}
