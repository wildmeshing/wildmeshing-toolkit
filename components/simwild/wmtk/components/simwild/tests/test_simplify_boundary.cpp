#include <catch2/catch_test_macros.hpp>

#include <jse/jse.h>
#include <simwild_spec.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::simwild;

namespace {

/// A flat, open n x n grid on [0, 1]^2: a square outline of 4 * (n - 1) boundary edges.
std::filesystem::path write_open_patch(const int n)
{
    // A name of its own, so concurrent runs sharing the temp directory do not collide.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("wmtk_simwild_open_patch_" + std::to_string(std::random_device{}()) + ".obj");
    std::ofstream out(path);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            out << "v " << double(i) / (n - 1) << " " << double(j) / (n - 1) << " 0\n";
        }
    }
    const auto id = [n](int i, int j) { return i * n + j + 1; };
    for (int i = 0; i + 1 < n; ++i) {
        for (int j = 0; j + 1 < n; ++j) {
            out << "f " << id(i, j) << " " << id(i + 1, j) << " " << id(i + 1, j + 1) << "\n";
            out << "f " << id(i, j) << " " << id(i + 1, j + 1) << " " << id(i, j + 1) << "\n";
        }
    }
    return path;
}

nlohmann::json params(const std::filesystem::path& input, const bool boundary_envelope)
{
    nlohmann::json j;
    j["application"] = "simwild";
    j["input"] = {input.string()};
    j["preserve_topology"] = false; // the only route that simplifies before insertion
    j["eps_simplify_rel"] = 1e-2;
    j["simplify_boundary_envelope"] = boundary_envelope;
    const auto spec = jse::embed::wmtk_simwild_spec::simwild_spec::spec();
    jse::JSE engine;
    REQUIRE(engine.verify_json(j, spec));
    return engine.inject_defaults(j, spec);
}

/// The boundary edges of the simplified surface read_mesh hands to the optimizer's envelope.
std::vector<std::array<Vector3d, 2>> boundary_edges(const InputData& data)
{
    std::map<std::pair<int, int>, int> count;
    for (int f = 0; f < data.F_envelope.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const int a = data.F_envelope(f, k), b = data.F_envelope(f, (k + 1) % 3);
            ++count[{std::min(a, b), std::max(a, b)}];
        }
    }
    std::vector<std::array<Vector3d, 2>> out;
    for (const auto& [e, c] : count) {
        if (c == 1) {
            out.push_back({{data.V_envelope.row(e.first), data.V_envelope.row(e.second)}});
        }
    }
    return out;
}

double dist_to_square_outline(const Vector3d& p)
{
    const double x = std::clamp(p[0], 0.0, 1.0), y = std::clamp(p[1], 0.0, 1.0);
    const double in_plane = std::min({x, 1 - x, y, 1 - y});
    const double off = (Vector3d(x, y, 0) - p).norm();
    return std::sqrt(in_plane * in_plane + off * off);
}

} // namespace

TEST_CASE("simwild-simplification-boundary-envelope", "[simwild][simplify][boundary]")
{
    // With topology preservation off the input is simplified before insertion, and its open
    // boundary either frozen or held in a tube of order2_envelope_ratio * eps_simplify around the
    // input's boundary edges.
    const int n = 21;
    const std::filesystem::path path = write_open_patch(n);

    SECTION("frozen when simplify_boundary_envelope is off")
    {
        const InputData data = read_mesh({path.string()}, "", params(path, false));
        // Every outline vertex survives, so the outline keeps all of its edges.
        CHECK(boundary_edges(data).size() == size_t(4 * (n - 1)));
    }
    SECTION("coarsened in the tube when it is on")
    {
        const nlohmann::json j = params(path, true);
        const InputData data = read_mesh({path.string()}, "", j);
        const double diag = std::sqrt(2.0);
        const double r = double(j["eps_simplify_rel"]) * diag * double(j["order2_envelope_ratio"]);
        const auto be = boundary_edges(data);
        REQUIRE(!be.empty());
        for (const auto& e : be) {
            for (int k = 0; k <= 16; ++k) {
                CHECK(dist_to_square_outline(e[0] + (e[1] - e[0]) * (k / 16.0)) <= r);
            }
        }
        CHECK(be.size() < size_t(4 * (n - 1)) / 2);
    }

    std::filesystem::remove(path);
}
