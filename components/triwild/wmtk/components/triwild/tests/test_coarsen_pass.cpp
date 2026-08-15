#include <wmtk/TriMesh.h>
#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/Types.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

/**
 * @brief A w x h grid of unit squares, each split into two triangles.
 *
 * Every vertex is interior-or-boundary of one flat sheet with no surface tags, so no envelope
 * is consulted anywhere and the coarsening pass is exercised on geometry alone.
 */
void build_grid(TriWildMesh& mesh, const size_t w, const size_t h)
{
    const auto vid = [w](size_t i, size_t j) { return j * (w + 1) + i; };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
            tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    const size_t nv = (w + 1) * (h + 1);
    mesh.init(nv, tris);

    mesh.m_vertex_attribute.resize(nv);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());

    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            auto& va = mesh.m_vertex_attribute[vid(i, j)];
            va.m_posf = Vector2d(double(i), double(j));
            va.m_pos = to_rational(va.m_posf);
            va.m_is_rounded = true;
            va.m_is_on_surface = false;
            va.m_sizing_scalar = 1.0;
        }
    }
    for (size_t f = 0; f < tris.size(); ++f) {
        mesh.m_face_attribute[f].m_quality = mesh.get_quality(mesh.oriented_tri_vids(f));
    }
}

/**
 * @brief A triangular lattice of unit equilateral triangles, w+1 vertices per row.
 *
 * Every face sits exactly at the AMIPS minimum, so no interior collapse can leave the worst
 * element of its region where it found it -- which is what makes "this one was rejected" a
 * property of the geometry rather than a hope. A square grid does NOT have that property: its
 * right isoceles triangles score 2.309 against an optimum of 2, so there is slack and
 * collapsing genuinely improves the mesh, which is what the next test relies on.
 */
void build_lattice(TriWildMesh& mesh, const size_t w, const size_t h)
{
    const double dy = std::sqrt(3.) / 2.;
    const auto vid = [w](size_t i, size_t j) { return j * (w + 1) + i; };
    const auto pos = [&](size_t i, size_t j) {
        return Vector2d(double(i) + (j % 2 ? 0.5 : 0.0), double(j) * dy);
    };

    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j + 1 <= h; ++j) {
        for (size_t i = 0; i + 1 <= w; ++i) {
            if (j % 2 == 0) {
                tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
                tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
            } else {
                tris.push_back({{vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)}});
                tris.push_back({{vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)}});
            }
        }
    }

    const size_t nv = (w + 1) * (h + 1);
    mesh.init(nv, tris);
    mesh.m_vertex_attribute.resize(nv);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());

    for (size_t j = 0; j <= h; ++j) {
        for (size_t i = 0; i <= w; ++i) {
            auto& va = mesh.m_vertex_attribute[vid(i, j)];
            va.m_posf = pos(i, j);
            va.m_pos = to_rational(va.m_posf);
            va.m_is_rounded = true;
            va.m_is_on_surface = false;
            va.m_sizing_scalar = 1.0;
        }
    }
    for (size_t f = 0; f < tris.size(); ++f) {
        mesh.m_face_attribute[f].m_quality = mesh.get_quality(mesh.oriented_tri_vids(f));
    }
}

/// Everything the coarsening composite is allowed to write, in one comparable value.
struct MeshSnapshot
{
    std::vector<std::array<size_t, 3>> faces;
    std::vector<double> qualities;
    std::vector<Vector2d> positions;
    std::vector<bool> rounded;

    bool operator==(const MeshSnapshot& o) const
    {
        return faces == o.faces && qualities == o.qualities && rounded == o.rounded &&
               positions.size() == o.positions.size() &&
               std::equal(
                   positions.begin(),
                   positions.end(),
                   o.positions.begin(),
                   [](const Vector2d& a, const Vector2d& b) { return a == b; });
    }
};

MeshSnapshot snapshot(const TriWildMesh& mesh)
{
    MeshSnapshot s;
    for (const TriMesh::Tuple& f : mesh.get_faces()) {
        const size_t fid = f.fid(mesh);
        s.faces.push_back(mesh.oriented_tri_vids(fid));
        s.qualities.push_back(mesh.m_face_attribute[fid].m_quality);
    }
    for (const TriMesh::Tuple& v : mesh.get_vertices()) {
        const size_t vid = v.vid(mesh);
        s.positions.push_back(mesh.m_vertex_attribute[vid].m_posf);
        s.rounded.push_back(mesh.m_vertex_attribute[vid].m_is_rounded);
    }
    return s;
}

Parameters make_params()
{
    Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(20, 20));
    params.coarsen_pass = true;
    params.coarsen_max_rounds = 2;
    params.coarsen_smooth_ring = 2;
    return params;
}

} // namespace

TEST_CASE("triwild-coarsen-rejected-collapse-leaves-no-trace", "[triwild_operation][coarsen]")
{
    // One collapse, driven directly, on an interior edge of an equilateral lattice: every face
    // there is exactly at the AMIPS minimum, so whatever the collapse produces is worse and the
    // composite has to refuse it. The mesh afterwards must be indistinguishable from the mesh
    // before -- connectivity, positions and qualities reverting together is the whole claim the
    // composite rests on, and a partial undo shows up as a difference in any of the three.
    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 0;

    TriWildMesh mesh(params, params.eps, 0);
    build_lattice(mesh, 6, 6);
    const auto vid = [](size_t i, size_t j) { return j * 7 + i; };

    const MeshSnapshot before = snapshot(mesh);
    REQUIRE(before.faces.size() == 72);
    // Both endpoints have a full six-face disk, so neither is on the lattice's ragged border,
    // where deleting a corner face is legitimately free and the collapse would be accepted.
    REQUIRE(mesh.get_one_ring_fids_for_vertex(vid(3, 3)).size() == 6);
    REQUIRE(mesh.get_one_ring_fids_for_vertex(vid(4, 3)).size() == 6);

    const auto [edge, eid] = mesh.tuple_from_edge({{vid(3, 3), vid(4, 3)}});
    REQUIRE(edge.is_valid(mesh));

    std::vector<TriMesh::Tuple> new_tris;
    const bool accepted = mesh.coarsen_collapse_edge(edge, new_tris);

    REQUIRE_FALSE(accepted);
    REQUIRE(snapshot(mesh) == before);
    // Rollback restores the face hashes too, so a tuple minted before the attempt is usable
    // again -- without that, the executor would drop work that never actually happened.
    REQUIRE(edge.is_valid(mesh));
}

TEST_CASE("triwild-coarsen-removes-what-the-mesh-does-not-need", "[triwild_operation][coarsen]")
{
    // The other side of it: a square grid's right isoceles triangles score above the AMIPS
    // optimum, so there is slack, and the pass is supposed to spend it on removing elements
    // rather than leaving them there -- without the max energy rising.
    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 2;
    params.coarsen_global_smoothing_passes = 1;

    TriWildMesh mesh(params, params.eps, 0);
    build_grid(mesh, 8, 8);

    mesh.coarsen_mesh();

    REQUIRE(mesh.m_coarsen_stats.accepted > 0);
    REQUIRE(mesh.m_coarsen_stats.cells_after < mesh.m_coarsen_stats.cells_before);
    REQUIRE(mesh.m_coarsen_stats.max_energy_after <= mesh.m_coarsen_stats.max_energy_before);
}

TEST_CASE("triwild-coarsen-never-raises-max-energy", "[triwild_operation][coarsen]")
{
    // The invariant the pass exists to hold, with the local smoothing switched on so the
    // accept/reject decision runs against a re-smoothed neighbourhood.
    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 2;
    params.coarsen_global_smoothing_passes = 1;

    TriWildMesh mesh(params, params.eps, 0);
    build_grid(mesh, 8, 8);

    const size_t faces_before = mesh.get_faces().size();
    const double max_before = std::get<0>(mesh.get_max_avg_energy());

    mesh.coarsen_mesh();

    REQUIRE(mesh.m_coarsen_stats.cells_before == faces_before);
    REQUIRE(mesh.m_coarsen_stats.max_energy_before == max_before);
    REQUIRE(mesh.m_coarsen_stats.max_energy_after <= mesh.m_coarsen_stats.max_energy_before);
    REQUIRE(mesh.m_coarsen_stats.cells_after <= mesh.m_coarsen_stats.cells_before);

    // The stored qualities must still describe the geometry -- a partial rollback would show
    // up here as a face whose cached quality belongs to a different configuration entirely.
    //
    // Compared to a relative tolerance, NOT exactly. collapse_edge_after stores the value
    // collapse_edge_before computed from the PREDICTED post-collapse vids, substituting v2 for
    // v1 in place; recomputing here reads the vids the connectivity actually ended up with,
    // which may be a rotation of that triple. AMIPS is rotation invariant mathematically but
    // not bit-for-bit, so an exact comparison here asserts something the code never promised --
    // and it does fail, rarely, on whichever face happens to differ.
    for (const TriMesh::Tuple& f : mesh.get_faces()) {
        const size_t fid = f.fid(mesh);
        const double stored = mesh.m_face_attribute[fid].m_quality;
        const double recomputed = mesh.get_quality(mesh.oriented_tri_vids(fid));
        REQUIRE(std::abs(stored - recomputed) <= 1e-9 * std::max(1.0, std::abs(recomputed)));
    }
}

TEST_CASE("triwild-coarsen-pass-off-is-a-no-op", "[triwild_operation][coarsen]")
{
    Parameters params = make_params();
    params.coarsen_pass = false;

    TriWildMesh mesh(params, params.eps, 0);
    build_grid(mesh, 6, 6);

    const MeshSnapshot before = snapshot(mesh);
    REQUIRE(mesh.coarsen_mesh() == 0);
    REQUIRE(snapshot(mesh) == before);
    REQUIRE(mesh.m_coarsen_stats.cells_before == 0); // untouched, so the report reads zero
}

TEST_CASE("triwild-coarsen-bounded-stops-at-the-target-length", "[triwild_operation][coarsen]")
{
    // The gate the coarsen_unbounded option controls. Every edge of the grid is length 1, so
    // driving the collapse threshold below that must stop the pass dead -- and lifting it with
    // coarsen_unbounded must bring the same mesh straight back to coarsening.
    const auto run = [](bool unbounded, double collapsing_l2) {
        Parameters params = make_params();
        params.coarsen_unbounded = unbounded;
        TriWildMesh mesh(params, params.eps, 0);
        build_grid(mesh, 8, 8);
        // init() derived this from the bounding box; override it after the mesh is built so the
        // threshold, not the geometry, is what differs between the two arms.
        mesh.m_params.collapsing_l2 = collapsing_l2;
        mesh.coarsen_mesh();
        return mesh.m_coarsen_stats.accepted;
    };

    REQUIRE(run(false, 0.25) == 0); // every edge is at 1.0, well past a 0.5 threshold
    REQUIRE(run(true, 0.25) > 0); // same mesh, same threshold, gate lifted
    REQUIRE(run(false, 4.0) > 0); // threshold above the edge length: the gate lets them through
}

namespace {

/**
 * @brief TriWild with a per-face quality target, standing in for SimWild's quality_field.
 *
 * TriWild's own target is uniform, so raw and relative quality order its faces identically and
 * nothing in the shared code can tell them apart. SimWild's is per tag, and that is where they
 * come apart -- so the seam needs a mesh whose target actually varies to be tested at all.
 */
class TargetedTriWildMesh : public TriWildMesh
{
public:
    using TriWildMesh::TriWildMesh;

    std::vector<double> m_target;

    double quality_rel(const size_t fid) const override
    {
        return m_face_attribute.at(fid).m_quality / m_target.at(fid);
    }
};

} // namespace

TEST_CASE("triwild-coarsen-judges-on-the-per-face-target", "[triwild_operation][coarsen]")
{
    // The coarsening accept test compares one face's quality against another's, which is only
    // meaningful once both are expressed relative to what each face is required to reach. This
    // pins that it goes through quality_rel rather than reading the raw quality directly.
    //
    // Same lattice and same edge as the rejection test above, so the geometry is already known
    // to produce faces worse than the ones it replaces -- raw, this collapse is refused.
    const auto build = [](TargetedTriWildMesh& mesh) {
        build_lattice(mesh, 6, 6);
        const auto vid = [](size_t i, size_t j) { return j * 7 + i; };
        REQUIRE(mesh.get_one_ring_fids_for_vertex(vid(3, 3)).size() == 6);
        REQUIRE(mesh.get_one_ring_fids_for_vertex(vid(4, 3)).size() == 6);
        return std::get<0>(mesh.tuple_from_edge({{vid(3, 3), vid(4, 3)}}));
    };

    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 0; // the collapse is the only thing writing quality

    // Arm 1: uniform target. Every face is at the AMIPS optimum, so whatever the collapse
    // produces is worse, and the pass refuses it -- exactly as it does without any target at all.
    {
        TargetedTriWildMesh mesh(params, params.eps, 0);
        const TriMesh::Tuple edge = build(mesh);
        mesh.m_target.assign(mesh.tri_capacity(), 1.0);

        std::vector<TriMesh::Tuple> new_tris;
        REQUIRE_FALSE(mesh.coarsen_collapse_edge(edge, new_tris));
    }

    // Arm 2: the same collapse, accepted -- because the faces it degrades are the ones allowed
    // to be bad, and the region's worst RELATIVE face is one the collapse never touches. Read
    // raw, this is the shielding that makes the unfixed comparison unsound: the worst raw face
    // rises and the test still has to notice. Read relative, the ceiling belongs to a face that
    // did not move, so the collapse is genuinely free and taking it is correct.
    {
        TargetedTriWildMesh mesh(params, params.eps, 0);
        const TriMesh::Tuple edge = build(mesh);
        const auto vid = [](size_t i, size_t j) { return j * 7 + i; };

        const double q0 = mesh.m_face_attribute[0].m_quality; // uniform on an equilateral lattice
        mesh.m_target.assign(mesh.tri_capacity(), q0 / 10.); // untouched faces sit at rel 10
        for (const size_t endpoint : {vid(3, 3), vid(4, 3)}) {
            for (const size_t fid : mesh.get_one_ring_fids_for_vertex(endpoint)) {
                mesh.m_target[fid] = 100. * q0; // whatever the collapse does here stays under 1
            }
        }

        std::vector<TriMesh::Tuple> new_tris;
        REQUIRE(mesh.coarsen_collapse_edge(edge, new_tris));
    }
}
