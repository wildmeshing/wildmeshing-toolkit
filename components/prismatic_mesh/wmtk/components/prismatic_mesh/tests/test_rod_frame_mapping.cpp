#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <wmtk/components/prismatic_mesh/rod_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::Vector3d;

namespace {
constexpr double pi = 3.14159265358979323846;
PrismaticMeshInput
field(const std::vector<Vector3d>& points, const std::vector<double>& phases, bool closed = false)
{
    PrismaticMeshInput input;
    input.vertices.resize(points.size(), 3);
    input.rod_state = std::make_shared<RodMeshState>();
    auto& state = *input.rod_state;
    for (size_t i = 0; i < points.size(); ++i) {
        input.vertices.row(i) = points[i].transpose();
        input.source_vertex_ids.push_back(100 + i);
    }
    std::vector<size_t> path;
    for (size_t i = 0; i < phases.size(); ++i) {
        const size_t j = (i + 1) % points.size();
        const auto t = (points[j] - points[i]).normalized();
        const auto rotation = (Eigen::Quaterniond::FromTwoVectors(Vector3d::UnitZ(), t) *
                               Eigen::AngleAxisd(phases[i], Vector3d::UnitZ()))
                                  .normalized();
        state.input_edges.push_back({i, j});
        state.directed_input_edges.push_back({i, j});
        state.edge_frames.push_back({rotation, 6, true});
        path.push_back(i);
    }
    REQUIRE(phases.size() == points.size() - (closed ? 0 : 1));
    state.rods.push_back(std::move(path));
    return input;
}
Vector3d vector_from_json(const nlohmann::json& a)
{
    return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()};
}
} // namespace

TEST_CASE("rod mapping distributes endpoint twist by physical arc length", "[rod][frame_mapping]")
{
    auto input = field({{0, 0, 0}, {0, 0, 1}, {0, 0, 3}, {0, 0, 6}}, {0, .22, .4});
    const auto original = input.rod_state->edge_frames;
    transport_rod_frame_mapping(input);
    const auto& state = *input.rod_state;
    const auto& report = state.report["frame_mapping"]["paths"][0];
    REQUIRE(report["completed"] == true);
    REQUIRE(std::abs(report["span_length"].get<double>() - 4) < 1e-12);
    REQUIRE(std::abs(report["delta_theta_radians"].get<double>() - .4) < 1e-12);
    REQUIRE(std::abs(report["weighted_twist_energy"].get<double>() - .04) < 1e-12);
    REQUIRE(report["closure_error_radians"].get<double>() < 1e-12);
    REQUIRE(report["maximum_twist_density_error"].get<double>() < 1e-12);
    REQUIRE(state.edge_frames.front().rotation.angularDistance(original.front().rotation) < 1e-12);
    REQUIRE(state.edge_frames.back().rotation.angularDistance(original.back().rotation) < 1e-12);
    // Edge midpoints are at z={.5,2,4.5}: the middle phase is .4*(1.5/4).
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(.15, Vector3d::UnitZ()));
    REQUIRE(state.edge_frames[1].rotation.angularDistance(expected) < 1e-12);
    REQUIRE(state.edge_frames[1].rotation.angularDistance(original[1].rotation) > .069);
    for (const auto& f : state.edge_frames)
        REQUIRE((f.rotation * Vector3d::UnitZ() - Vector3d::UnitZ()).norm() < 1e-12);
    const auto mean = state.edge_frames[0].rotation.slerp(.5, state.edge_frames[1].rotation);
    REQUIRE(state.vertex_rotations.at(1).angularDistance(mean) < 1e-12);
}

TEST_CASE(
    "rod mapping rounds only the endpoint and retains its degree lattice",
    "[rod][frame_mapping]")
{
    auto input = field({{0, 0, 0}, {0, 0, 1}, {0, 0, 3}, {0, 0, 6}}, {0, .45, .9});
    const auto end = input.rod_state->edge_frames.back().rotation;
    transport_rod_frame_mapping(input);
    const auto& state = *input.rod_state;
    const auto& report = state.report["frame_mapping"]["paths"][0];
    const double delta = .9 - pi / 3;
    REQUIRE(std::abs(report["delta_theta_radians"].get<double>() - delta) < 1e-12);
    REQUIRE(report["terminal_degree_shift"] == 5);
    REQUIRE(report["frames"].size() == 3);
    REQUIRE(std::abs(report["frames"][1]["mapped_phase_radians"].get<double>() + .45) < 1e-12);
    const Eigen::Quaterniond middle(Eigen::AngleAxisd(delta * 1.5 / 4, Vector3d::UnitZ()));
    REQUIRE(state.edge_frames[1].rotation.angularDistance(middle) < 1e-12);
    for (int k = 0; k < 6; ++k) {
        const Vector3d actual =
            state.edge_frames.back().rotation *
            (Eigen::AngleAxisd(k * pi / 3, Vector3d::UnitZ()) * Vector3d::UnitX());
        const Vector3d expected =
            end * (Eigen::AngleAxisd((k - 1) * pi / 3, Vector3d::UnitZ()) * Vector3d::UnitX());
        REQUIRE((actual - expected).norm() < 1e-12);
    }
}

TEST_CASE("rod ray mapping passes through the quaternion-mean corner plane", "[rod][frame_mapping]")
{
    auto input = field({{0, 0, 0}, {0, 0, 2}, {2, 0, 2}}, {.3, .7});
    transport_rod_frame_mapping(input);
    const auto& report = input.rod_state->report["frame_mapping"]["paths"][0];
    REQUIRE(report["completed"] == true);
    REQUIRE(report["transitions"].size() == 1);
    const auto& step = report["transitions"][0];
    const Vector3d ray = vector_from_json(step["vertex_ray"]);
    const Vector3d normal = vector_from_json(step["vertex_normal"]);
    REQUIRE(std::abs(ray.dot(normal)) < 1e-12);
    // Independent line/plane intersection reference for this bent, twisted pair.
    const Vector3d expected{0, .3056650504697825, -.9521391058670499};
    REQUIRE((vector_from_json(step["next_edge_ray"]) - expected).norm() < 1e-12);
    const auto& state = *input.rod_state;
    const auto mean = state.edge_frames[0].rotation.slerp(.5, state.edge_frames[1].rotation);
    REQUIRE(state.vertex_rotations.at(1).angularDistance(mean) < 1e-12);
}

TEST_CASE(
    "a closed rod distributes holonomy without duplicating its first frame",
    "[rod][frame_mapping]")
{
    auto input = field(
        {{0, 0, 0}, {2, 0, 0}, {2, 1, 1}, {1, 2, .3}, {-.5, 1, 1.2}},
        {0, .13, -.07, .23, -.11},
        true);
    const auto first = input.rod_state->edge_frames.front().rotation;
    transport_rod_frame_mapping(input);
    const auto& state = *input.rod_state;
    const auto& report = state.report["frame_mapping"]["paths"][0];
    INFO(report.dump(2));
    REQUIRE(report["completed"] == true);
    REQUIRE(report["closed"] == true);
    REQUIRE(report["frames"].size() == 5);
    REQUIRE(report["transitions"].size() == 5);
    REQUIRE(std::abs(report["delta_theta_radians"].get<double>()) > 1e-3);
    REQUIRE(report["closure_error_radians"].get<double>() < 1e-12);
    REQUIRE(report["maximum_twist_density_error"].get<double>() < 1e-12);
    REQUIRE(state.edge_frames.front().rotation.angularDistance(first) < 1e-12);
    double length = 0;
    for (size_t i = 0; i < 5; ++i) {
        const Vector3d d = input.vertices.row((i + 1) % 5) - input.vertices.row(i);
        length += d.norm();
        REQUIRE(
            (state.edge_frames[i].rotation * Vector3d::UnitZ() - d.normalized()).norm() < 1e-12);
    }
    REQUIRE(std::abs(report["span_length"].get<double>() - length) < 1e-12);
}

TEST_CASE(
    "a parallel corner projection retains its initial valid frame field",
    "[rod][frame_mapping]")
{
    auto input = field({{0, 0, 0}, {0, 0, 1}, {0, 0, .1}}, {0, 0});
    const auto before = input.rod_state->edge_frames;
    transport_rod_frame_mapping(input);
    const auto& report = input.rod_state->report["frame_mapping"]["paths"][0];
    REQUIRE(report["completed"] == false);
    REQUIRE(report["reason"] == "parallel_or_degenerate_projection");
    for (size_t i = 0; i < before.size(); ++i)
        REQUIRE(
            input.rod_state->edge_frames[i].rotation.angularDistance(before[i].rotation) < 1e-12);
}
