#include <wmtk/TetMesh.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

/**
 * @brief A w x h x d grid of unit cubes, each cut into six tets (Kuhn / Freudenthal).
 *
 * The 3D counterpart of the 2D tests' grid, and it has the same useful property: the six tets
 * of a cube are well formed but nowhere near the AMIPS optimum, so there is slack for the
 * coarsening pass to spend, and the invariant tests below have something to bite on.
 *
 * No vertex is tagged on-surface, so no envelope is consulted anywhere and the pass is
 * exercised on geometry alone.
 */
void build_cube_grid(TetWildMesh& mesh, const size_t w, const size_t h, const size_t d)
{
    const auto vid = [w, h](size_t i, size_t j, size_t k) {
        return (k * (h + 1) + j) * (w + 1) + i;
    };
    const auto pos = [](size_t i, size_t j, size_t k) {
        return Vector3d(double(i), double(j), double(k));
    };

    // The six tets of the unit cube, by corner bitmask (bit0 = x, bit1 = y, bit2 = z).
    static const std::array<std::array<int, 4>, 6> KUHN = {{
        {{0, 1, 3, 7}},
        {{0, 1, 5, 7}},
        {{0, 4, 5, 7}},
        {{0, 2, 3, 7}},
        {{0, 2, 6, 7}},
        {{0, 4, 6, 7}},
    }};

    std::vector<std::array<size_t, 4>> tets;
    for (size_t k = 0; k < d; ++k) {
        for (size_t j = 0; j < h; ++j) {
            for (size_t i = 0; i < w; ++i) {
                const auto corner = [&](int b) {
                    return vid(i + (b & 1), j + ((b >> 1) & 1), k + ((b >> 2) & 1));
                };
                const auto corner_pos = [&](int b) {
                    return pos(i + (b & 1), j + ((b >> 1) & 1), k + ((b >> 2) & 1));
                };
                for (const auto& t : KUHN) {
                    std::array<size_t, 4> vs = {
                        {corner(t[0]), corner(t[1]), corner(t[2]), corner(t[3])}};
                    // Orient positively rather than trusting the table's winding.
                    const Vector3d a = corner_pos(t[0]), b = corner_pos(t[1]);
                    const Vector3d c = corner_pos(t[2]), e = corner_pos(t[3]);
                    if ((b - a).cross(c - a).dot(e - a) < 0) {
                        std::swap(vs[1], vs[2]);
                    }
                    tets.push_back(vs);
                }
            }
        }
    }

    const size_t nv = (w + 1) * (h + 1) * (d + 1);
    mesh.init(nv, tets);

    std::vector<VertexAttributes> va(nv);
    for (size_t k = 0; k <= d; ++k) {
        for (size_t j = 0; j <= h; ++j) {
            for (size_t i = 0; i <= w; ++i) {
                auto& a = va[vid(i, j, k)];
                a.m_posf = pos(i, j, k);
                a.m_pos = to_rational(a.m_posf);
                a.m_is_rounded = true;
                a.m_is_on_surface = false;
                a.m_sizing_scalar = 1.0;
            }
        }
    }
    std::vector<TetAttributes> ta(tets.size());
    mesh.create_mesh_attributes(va, ta);
    for (size_t t = 0; t < tets.size(); ++t) {
        mesh.m_tet_attribute[t].m_quality = mesh.get_quality(mesh.oriented_tet_vids(t));
    }
}

/// Everything the coarsening composite is allowed to write, in one comparable value.
struct MeshSnapshot
{
    std::vector<std::array<size_t, 4>> cells;
    std::vector<double> qualities;
    std::vector<Vector3d> positions;
    std::vector<bool> rounded;

    bool operator==(const MeshSnapshot& o) const
    {
        return cells == o.cells && qualities == o.qualities && rounded == o.rounded &&
               positions.size() == o.positions.size() &&
               std::equal(
                   positions.begin(),
                   positions.end(),
                   o.positions.begin(),
                   [](const Vector3d& a, const Vector3d& b) { return a == b; });
    }
};

MeshSnapshot snapshot(TetWildMesh& mesh)
{
    MeshSnapshot s;
    for (const TetMesh::Tuple& t : mesh.get_tets()) {
        const size_t tid = t.tid(mesh);
        s.cells.push_back(mesh.oriented_tet_vids(tid));
        s.qualities.push_back(mesh.m_tet_attribute[tid].m_quality);
    }
    for (const TetMesh::Tuple& v : mesh.get_vertices()) {
        const size_t vid = v.vid(mesh);
        s.positions.push_back(mesh.m_vertex_attribute[vid].m_posf);
        s.rounded.push_back(mesh.m_vertex_attribute[vid].m_is_rounded);
    }
    return s;
}

Parameters make_params()
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(20, 20, 20));
    params.coarsen_pass = true;
    params.coarsen_max_rounds = 2;
    params.coarsen_smooth_ring = 2;
    return params;
}

} // namespace

TEST_CASE("tetwild-coarsen-rejected-collapse-leaves-no-trace", "[tetwild_operation][coarsen]")
{
    // The claim the composite rests on: when the region test refuses a collapse, the
    // connectivity, the positions, the cell qualities AND the tet hashes all revert together,
    // so a partial undo shows up as a difference in any of the four.
    //
    // Unlike 2D there is no regular tetrahedral tiling to guarantee that a given collapse is
    // refused -- no regular tet tiles space -- so rather than assert which edge gets refused,
    // walk edges until the pass refuses one and check that one exactly. The final REQUIRE
    // keeps the test from passing vacuously if nothing is ever refused.
    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 0;

    auto env = std::make_shared<SampleEnvelope>();
    TetWildMesh mesh(params, env, 0);
    build_cube_grid(mesh, 3, 3, 3);

    size_t rejected = 0;
    size_t checked = 0;
    for (const TetMesh::Tuple& e : mesh.get_edges()) {
        if (rejected >= 3) {
            break;
        }
        const MeshSnapshot before = snapshot(mesh);
        std::vector<TetMesh::Tuple> new_tets;
        const bool accepted = mesh.coarsen_collapse_edge(e, new_tets);
        ++checked;
        if (accepted) {
            continue; // the mesh moved; carry on from the new state
        }
        ++rejected;
        REQUIRE(snapshot(mesh) == before);
        // Rollback restores the tet hashes too, so a tuple minted before the attempt is usable
        // again -- without that, the executor would drop work that never actually happened.
        REQUIRE(e.is_valid(mesh));
    }
    INFO("checked " << checked << " edges");
    REQUIRE(rejected > 0);
}

TEST_CASE("tetwild-coarsen-never-raises-max-energy", "[tetwild_operation][coarsen]")
{
    // The invariant the pass exists to hold, with the local smoothing switched on so the
    // accept/reject decision runs against a re-smoothed neighbourhood.
    Parameters params = make_params();
    params.coarsen_local_smoothing_passes = 2;
    params.coarsen_global_smoothing_passes = 1;

    auto env = std::make_shared<SampleEnvelope>();
    TetWildMesh mesh(params, env, 0);
    build_cube_grid(mesh, 4, 4, 4);

    const size_t cells_before = mesh.get_tets().size();
    const double max_before = std::get<0>(mesh.get_max_avg_energy());

    mesh.coarsen_mesh();

    REQUIRE(mesh.m_coarsen_stats.cells_before == cells_before);
    REQUIRE(mesh.m_coarsen_stats.max_energy_before == max_before);
    REQUIRE(mesh.m_coarsen_stats.max_energy_after <= mesh.m_coarsen_stats.max_energy_before);
    REQUIRE(mesh.m_coarsen_stats.cells_after <= mesh.m_coarsen_stats.cells_before);
    REQUIRE(mesh.m_coarsen_stats.accepted > 0);
    REQUIRE(mesh.m_coarsen_stats.cells_after < mesh.m_coarsen_stats.cells_before);

    // The stored qualities must still describe the geometry -- a partial rollback would show
    // up here as a cell whose cached quality belongs to a different configuration entirely.
    //
    // Compared to a relative tolerance, NOT exactly, for the same reason as the 2D twin:
    // collapse_edge_after stores what collapse_edge_before computed from the PREDICTED
    // post-collapse vids, and recomputing here reads the vids the connectivity ended up with,
    // which may be a permutation of that tuple. AMIPS is permutation invariant mathematically
    // but not bit-for-bit.
    for (const TetMesh::Tuple& t : mesh.get_tets()) {
        const size_t tid = t.tid(mesh);
        const double stored = mesh.m_tet_attribute[tid].m_quality;
        const double recomputed = mesh.get_quality(mesh.oriented_tet_vids(tid));
        REQUIRE(std::abs(stored - recomputed) <= 1e-9 * std::max(1.0, std::abs(recomputed)));
    }
}

TEST_CASE("tetwild-coarsen-pass-off-is-a-no-op", "[tetwild_operation][coarsen]")
{
    Parameters params = make_params();
    params.coarsen_pass = false;

    auto env = std::make_shared<SampleEnvelope>();
    TetWildMesh mesh(params, env, 0);
    build_cube_grid(mesh, 3, 3, 3);

    const MeshSnapshot before = snapshot(mesh);
    REQUIRE(mesh.coarsen_mesh() == 0);
    REQUIRE(snapshot(mesh) == before);
    REQUIRE(mesh.m_coarsen_stats.cells_before == 0); // untouched, so the report reads zero
}

TEST_CASE("tetwild-coarsen-bounded-stops-at-the-target-length", "[tetwild_operation][coarsen]")
{
    // The gate the coarsen_unbounded option controls. The grid's shortest edges are length 1,
    // so driving the collapse threshold below that must stop the pass dead -- and lifting it
    // with coarsen_unbounded must bring the same mesh straight back to coarsening.
    const auto run = [](bool unbounded, double collapsing_l2) {
        Parameters params = make_params();
        params.coarsen_unbounded = unbounded;
        auto env = std::make_shared<SampleEnvelope>();
        TetWildMesh mesh(params, env, 0);
        build_cube_grid(mesh, 3, 3, 3);
        // init() derived this from the bounding box; override it after the mesh is built so the
        // threshold, not the geometry, is what differs between the two arms.
        mesh.m_params.collapsing_l2 = collapsing_l2;
        mesh.coarsen_mesh();
        return mesh.m_coarsen_stats.accepted;
    };

    REQUIRE(run(false, 0.25) == 0); // every edge is at least 1.0, well past a 0.5 threshold
    REQUIRE(run(true, 0.25) > 0); // same mesh, same threshold, gate lifted
    REQUIRE(run(false, 16.0) > 0); // threshold above every edge: the gate lets them through
}
