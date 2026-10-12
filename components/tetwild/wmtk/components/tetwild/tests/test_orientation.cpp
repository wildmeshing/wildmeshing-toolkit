// The tracked surface carries the input's orientation (SurfaceTagAttributes::m_orientation)
// from the insertion through every operation, so the tracked-surface winding number is the
// input's own rather than a per-patch guess. The configurations below are the ones a guess gets
// wrong -- several components, one of them inside out, one nested in another, two touching
// face-to-face, one resting on another -- and the inputs it must repair or count once -- a
// triangle or a face flipped against its neighbours, a duplicated triangle -- side by side in
// ONE input, so a single insertion and
// optimization covers them all: the insertion's background lattice is diag/20 whatever the
// input, ~40k tets, and seven separate runs took the tetwild suite past its 1500 s ctest limit
// in a Windows Debug build.

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/components/tetwild/Parameters.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
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

/// Volume of the tets whose barycenter has x in [x0, x1) and whose winding number (tracked or
/// input) exceeds 1/2.
double volume_where(TetWildMesh& mesh, bool tracked, double x0, double x1)
{
    double vol = 0;
    for (const auto& t : mesh.get_tets()) {
        const size_t tid = t.tid(mesh);
        const auto& a = mesh.tet_finalize(tid);
        if ((tracked ? a.m_winding_number_tracked : a.m_winding_number_input) <= 0.5) continue;
        const auto vs = mesh.oriented_tet_vids(tid);
        const auto& p = mesh.m_vertex_attribute;
        const double cx =
            (p[vs[0]].m_posf[0] + p[vs[1]].m_posf[0] + p[vs[2]].m_posf[0] + p[vs[3]].m_posf[0]) / 4;
        if (cx < x0 || cx >= x1) continue;
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

struct Region
{
    const char* name;
    double x0, x1; // the configuration's slab of x
    double expected; // the solid's volume
    /// The input is consistently oriented there, so its own winding number gets the solid
    /// right and agrees with the tracked one. Not where the tracked surface repairs it.
    bool input_consistent = true;
};

/// The slab of x the tet with barycenter-x `cx` lies in, or nullptr.
const Region* region_of(const std::vector<Region>& regions, double cx)
{
    for (const auto& r : regions) {
        if (cx >= r.x0 && cx < r.x1) return &r;
    }
    return nullptr;
}

} // namespace

TEST_CASE("orientation-configurations", "[tetwild][orientation]")
{
    Soup s;
    // A box and an inside-out box: the second is a negative solid and must stay out.
    s.box({0, 0, 0}, {1, 1, 1});
    s.box({2, 0, 0}, {3, 1, 1}, true);
    // Nested boxes: winding number 2 inside the inner one, still inside.
    s.box({5, 0, 0}, {8, 3, 3});
    s.box({6, 1, 1}, {7, 2, 2});
    // Boxes sharing a face, on the same vertices: one coplanar group with both orientations,
    // which cancel to 0 there.
    s.box({10, 0, 0}, {11, 1, 1});
    s.box({11, 0, 0}, {12, 1, 1});
    // A box resting on a plate (Thingi10K 52564): its bottom lies inside the plate's top without
    // sharing vertices, so the facets under it belong to two coplanar groups.
    s.box({14, 0, 0}, {17, 3, 1});
    s.box({15, 1, 1}, {16, 2, 2});
    // A box resting along a plate's edge: sharing an edge makes the two opposite faces one
    // coplanar group of mixed orientation, the exact per-point path. (Thingi10K 104513 also has
    // a facet straddling an uncut edge between two of such a group's triangles; this does not
    // reproduce that.)
    s.box({19, 0, 0}, {22, 3, 1});
    s.box({19, 0, 1}, {22, 1, 2});
    // A box with one triangle of its top flipped: its coplanar neighbour now faces the other
    // way across their shared diagonal, which the arrangement need not cut. Repaired.
    s.box({25, 0, 0}, {26, 1, 1});
    std::swap(s.f[s.f.size() - 12 + 3][1], s.f[s.f.size() - 12 + 3][2]);
    // A box with a whole face flipped, both its triangles: no coplanar neighbour, so it is the
    // links across the box's edges that repair it.
    s.box({28, 0, 0}, {29, 1, 1});
    std::swap(s.f[s.f.size() - 12 + 10][1], s.f[s.f.size() - 12 + 10][2]);
    std::swap(s.f[s.f.size() - 12 + 11][1], s.f[s.f.size() - 12 + 11][2]);
    // A box with a triangle duplicated: the copies count once, so the surface stays closed.
    s.box({31, 0, 0}, {32, 1, 1});
    s.f.push_back(s.f[s.f.size() - 12 + 4]);
    const std::vector<Region> regions = {
        {"box + inside-out box", -1, 4, 1},
        {"nested boxes", 4, 9, 27},
        {"boxes sharing a face", 9, 13, 2},
        {"box resting on a plate", 13, 18, 10},
        {"box resting along a plate edge", 18, 23, 12},
        {"box with a flipped triangle", 24, 27, 1, false},
        {"box with a flipped face", 27, 30, 1, false},
        {"box with a duplicated triangle", 30, 33, 1, false}};

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
    std::vector<int8_t> tet_face_orientation;
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

    // Right after the insertion the tracked surface IS the input, repaired, so the oriented
    // surface is closed and the two winding numbers agree tet by tet where the input needed no
    // repair. Except on the arrangement's flat tets: lying in the surface, with their barycenter
    // on it, they have no winding number to agree on (the evaluation lands anywhere around 1/2),
    // and the optimizer removes them.
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
        const double cx =
            (p[vs[0]].m_posf[0] + p[vs[1]].m_posf[0] + p[vs[2]].m_posf[0] + p[vs[3]].m_posf[0]) / 4;
        const Region* r = region_of(regions, cx);
        if (r != nullptr && !r->input_consistent) continue;
        const auto& a = mesh.tet_finalize(tid);
        if (std::lround(a.m_winding_number_input) != std::lround(a.m_winding_number_tracked)) {
            ++disagree;
        }
    }
    logger().info(
        "[orientation] {} tets ({} flat), {} disagree after insertion",
        tets.size(),
        flat,
        disagree);
    CHECK(disagree == 0);
    for (const auto& r : regions) {
        const double vt = volume_where(mesh, true, r.x0, r.x1);
        INFO(r.name << ": tracked " << vt << " expected " << r.expected);
        CHECK(std::abs(vt - r.expected) < 1e-6 * r.expected);
    }

    // The operations must carry the orientation: still closed, and the tracked surface -- now
    // within eps of the input rather than on it -- still encloses the same solids.
    mesh.mesh_improvement(2);
    CHECK(mesh.tracked_surface_boundary().empty());
    compute_both_winding_numbers(mesh, s);
    for (const auto& r : regions) {
        const double vt = volume_where(mesh, true, r.x0, r.x1);
        const double vi = volume_where(mesh, false, r.x0, r.x1);
        logger().info(
            "[orientation] {}: after optimization, tracked {} input {} expected {}",
            r.name,
            vt,
            vi,
            r.expected);
        INFO(r.name);
        CHECK(std::abs(vt - r.expected) < 0.02 * r.expected);
        if (r.input_consistent) CHECK(std::abs(vi - r.expected) < 0.02 * r.expected);
    }

    // The finalization takes the whole input's winding number as the sum of the per-input ones,
    // with every box an input of its own here -- the inside-out one included, which as an input
    // of its own is oriented on its own, but still subtracts from the whole. It must be the one
    // evaluated on the whole input above, tet by tet (but the flat ones, whose barycenter is on
    // the surface).
    {
        const auto ts = mesh.get_tets();
        const Eigen::MatrixXd c = mesh.tet_barycenters(ts);
        std::vector<double> whole(ts.size());
        for (size_t i = 0; i < ts.size(); ++i) {
            whole[i] = mesh.tet_finalize(ts[i].tid(mesh)).m_winding_number_input;
        }
        const int n_inputs = int(s.f.size() / 12); // the duplicated triangle joins the last box
        std::vector<int> face_input(s.f.size());
        for (size_t f = 0; f < s.f.size(); ++f) face_input[f] = std::min(int(f / 12), n_inputs - 1);
        mesh.compute_input_winding_numbers(ts, c, s.v, s.f, face_input, n_inputs);
        double max_diff = 0;
        for (size_t i = 0; i < ts.size(); ++i) {
            const auto vs = mesh.oriented_tet_vids(ts[i]);
            const auto& p = mesh.m_vertex_attribute;
            const double vol = (p[vs[1]].m_posf - p[vs[0]].m_posf)
                                   .cross(p[vs[2]].m_posf - p[vs[0]].m_posf)
                                   .dot(p[vs[3]].m_posf - p[vs[0]].m_posf);
            if (std::abs(vol) < 1e-12) continue;
            const auto& a = mesh.tet_finalize(ts[i].tid(mesh));
            REQUIRE(a.m_winding_number_per_input.size() == size_t(n_inputs));
            max_diff = std::max(max_diff, std::abs(a.m_winding_number_input - whole[i]));
        }
        CHECK(max_diff < 1e-9);
    }
}
