#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <catch2/catch_test_macros.hpp>

#include <random>

using namespace wmtk;
using namespace wmtk::components;
using surface_map::SurfaceMap;
using surface_map::SurfacePoint;

namespace {

/// The history of an isotropic remeshing of `model`.
history::SurfaceHistory remesh(const std::string& model, int threads, double len_factor, int its)
{
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/" + model, V, F, 0);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);
    isotropic_remeshing::IsotropicRemeshing m(verts, threads);
    m.create_mesh(verts.size(), tris, {}, true, 0, 0);
    const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
    m.uniform_remeshing(len_factor * m.average_len_valen()[0], its);
    m.consolidate_mesh();
    m.stop_history();
    return rec->history();
}

Eigen::Vector3d position(
    const SurfacePoint& p,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F)
{
    const auto& f = F[p.face];
    return p.bary[0] * V[f[0]] + p.bary[1] * V[f[1]] + p.bary[2] * V[f[2]];
}

/// `n` random points strictly inside every live face.
std::vector<SurfacePoint> sample(const std::vector<uint8_t>& alive, int n, unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.02, 0.98);
    std::vector<SurfacePoint> pts;
    for (uint64_t f = 0; f < alive.size(); ++f) {
        if (!alive[f]) continue;
        for (int k = 0; k < n; ++k) {
            double a = u(rng), b = u(rng);
            if (a + b > 1) {
                a = 1 - a;
                b = 1 - b;
            }
            pts.push_back({f, Eigen::Vector3d(1 - a - b, a, b)});
        }
    }
    return pts;
}

/// Maps `pts` there and back and returns the largest 3D displacement; every point has to land
/// on a live face both ways.
double round_trip(
    const SurfaceMap& map,
    const std::vector<SurfacePoint>& pts,
    const bool from_input,
    size_t& warnings)
{
    const auto& V = from_input ? map.input_vertices() : map.output_vertices();
    const auto& F = from_input ? map.input_faces() : map.output_faces();
    const auto& other_alive = from_input ? map.output_face_alive() : map.input_face_alive();
    const auto there = from_input ? map.to_output(pts) : map.to_input(pts);
    warnings = map.map_warnings().total();
    for (const auto& p : there) {
        REQUIRE(p.face < other_alive.size());
        REQUIRE(other_alive[p.face]);
    }
    const auto back = from_input ? map.to_input(there) : map.to_output(there);
    warnings += map.map_warnings().total();
    double err = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(back[i].face < F.size());
        err = std::max(err, (position(back[i], V, F) - position(pts[i], V, F)).norm());
    }
    return err;
}

} // namespace

TEST_CASE("surface-map-round-trip-remeshing", "[surface_map]")
{
    // A closed manifold remeshed with the link condition on: every local map is a bijection, so
    // the map is one, and any point comes back to where it was.
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        const auto h = remesh("sphere.obj", threads, 0.6, 2);
        const SurfaceMap map(h);
        CHECK(map.build_warnings().total() == 0);
        CHECK(map.cells_by_kind()[size_t(history::EditKind::collapse_edge)] > 0);
        CHECK(map.cells_by_kind()[size_t(history::EditKind::swap_edge)] > 0);

        size_t warnings = 0;
        const double in_err = round_trip(map, sample(map.input_face_alive(), 3, 1), true, warnings);
        CHECK(warnings == 0);
        CHECK(in_err < 1e-9);

        const double out_err =
            round_trip(map, sample(map.output_face_alive(), 3, 2), false, warnings);
        CHECK(warnings == 0);
        CHECK(out_err < 1e-9);
    }
}

TEST_CASE("surface-map-round-trip-remeshing-bunny", "[surface_map][.]")
{
    // The bunny has boundary loops. Its boundary vertices are frozen, but the vertices splits
    // insert on boundary edges are not, and they can collapse along the boundary afterwards: the
    // chord case runs. Where such a vertex has a single triangle, the collapse flattens it onto
    // the chord -- the one way a 2D map breaks -- and the map reports it, with the atlas it
    // inverts, and nothing else. Hidden, a few seconds.
    const auto h = remesh("bunny.obj", 8, 0.8, 3);
    const SurfaceMap map(h);
    const auto& w = map.build_warnings();
    INFO(
        "not a disk " << w.not_a_disk << ", flat " << w.flat_triangle << ", inverted " << w.inverted
                      << ", merged " << w.merged_faces);
    CHECK(map.boundary_collapses() > 0);
    CHECK(w.not_a_disk == 0);
    CHECK(w.merged_faces == 0);
    CHECK(w.inverted <= w.flat_triangle);
    size_t warnings = 0;
    const double err = round_trip(map, sample(map.output_face_alive(), 1, 3), false, warnings);
    CHECK(warnings == 0);
    CHECK(err < 1e-9);
}
