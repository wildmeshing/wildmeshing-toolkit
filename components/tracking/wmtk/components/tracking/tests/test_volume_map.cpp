#include <wmtk/components/tracking/volume_geometry.hpp>
#include <wmtk/history/VolumeHistory.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <random>
#include <string>

using namespace wmtk;
using namespace wmtk::components::tracking;

TEST_CASE("volume-history-save-load", "[volume_map]")
{
    history::VolumeHistory h;
    h.regions = {{}, {0}, {0, 3}};
    h.surface_edit_offset = 2;
    history::VolumeEdit e;
    e.op = history::VolumeOp::collapse;
    e.first_edit = 7;
    e.n_edits = 1;
    e.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
    e.surface_before = {0, 1, history::kNoVertex, 2, 3};
    e.surface_after = {history::kNoVertex, 1, history::kNoVertex, 2, 3};
    e.moved = {{1, Eigen::Vector3d(0.5, 0, 0)}};
    e.tets = {{{0, 1, 2, 3}}, {{1, 2, 3, 4}}};
    e.regions = {1, 2};
    e.removed = {0};
    e.added = {{{4, 2, 1, 3}}};
    e.added_regions = {1};
    e.op_vertices = {0, 1};
    h.edits.push_back(e);
    const auto path = (std::filesystem::temp_directory_path() /
                       ("wmtk_volume_history_" + std::to_string(std::random_device{}()) + ".vol"))
                          .string();
    h.save(path);
    const auto g = history::VolumeHistory::load(path);
    std::filesystem::remove(path);
    CHECK(g.regions == h.regions);
    CHECK(g.surface_edit_offset == 2);
    REQUIRE(g.edits.size() == 1);
    const auto& f = g.edits[0];
    CHECK(f.op == e.op);
    CHECK(f.first_edit == 7);
    CHECK(f.positions == e.positions);
    CHECK(f.surface_before == e.surface_before);
    CHECK(f.surface_after == e.surface_after);
    CHECK(f.moved == e.moved);
    CHECK(f.tets == e.tets);
    CHECK(f.regions == e.regions);
    CHECK(f.removed == e.removed);
    CHECK(f.added == e.added);
    CHECK(f.added_regions == e.added_regions);
    CHECK(f.op_vertices == e.op_vertices);
    std::vector<uint32_t> ra;
    const auto after = f.tets_after(&ra);
    CHECK(after.size() == 2);
    CHECK(ra == std::vector<uint32_t>{2, 1});
    CHECK(f.positions_after()[1] == Eigen::Vector3d(0.5, 0, 0));
}

TEST_CASE("volume-chebyshev", "[volume_map]")
{
    // The unit cube's faces: the farthest point from all of them is its centre, at 1/2.
    std::vector<Eigen::Vector3d> N;
    std::vector<double> b;
    for (int k = 0; k < 3; ++k) {
        N.push_back(Eigen::Vector3d::Unit(k));
        b.push_back(1);
        N.push_back(-Eigen::Vector3d::Unit(k));
        b.push_back(0);
    }
    const auto r = volume::chebyshev(N, b, Eigen::Vector3d(5, -3, 2), 10, 10);
    REQUIRE(r);
    CHECK(std::abs(r->second - 0.5) < 1e-12);
    CHECK((r->first - Eigen::Vector3d::Constant(0.5)).norm() < 1e-12);
}

TEST_CASE("volume-spherical-fallback", "[volume_map]")
{
    // A cone from the origin over a band of the sphere around the equator, 300 degrees of it:
    // valid, but no point sees all its faces from inside, so the radial map cannot use it. The
    // spherical fallback keeps every vertex's distance from the apex and lays its direction out
    // again: a valid cone, star-shaped about a point next to the apex.
    using namespace volume;
    const int m = 30;
    Positions P = {Eigen::Vector3d::Zero()};
    for (int k = 0; k <= m; ++k) {
        const double phi = 300. / 180 * M_PI * k / m;
        const double r = 1 + 0.3 * std::sin(3 * phi); // radii vary
        P.push_back(r * Eigen::Vector3d(std::cos(phi), std::sin(phi), -0.2).normalized());
        P.push_back(r * Eigen::Vector3d(std::cos(phi), std::sin(phi), 0.2).normalized());
    }
    Tets T;
    for (int k = 0; k < m; ++k) {
        const Vid a0 = Vid(1 + 2 * k), b0 = a0 + 1, a1 = a0 + 2, b1 = a0 + 3;
        T.push_back({{0, a0, a1, b0}});
        T.push_back({{0, b0, a1, b1}});
    }
    const auto real = StarPatch::make(T, P);
    REQUIRE(real);
    CHECK_FALSE(centre_of(*real));

    const auto Q = spherical(T, P);
    REQUIRE(Q);
    const auto S = StarPatch::make(T, *Q);
    REQUIRE(S);
    const auto c = centre_of(*S);
    REQUIRE(c);
    CHECK(S->sees_all(*c));
    CHECK((*Q)[0] == P[0]); // the apex stays
}
