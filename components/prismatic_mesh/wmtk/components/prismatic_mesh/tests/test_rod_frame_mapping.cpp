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
PrismaticMeshInput sections(
    const std::vector<Vector3d>& centers,
    const std::vector<double>& phases,
    bool closed = false)
{
    PrismaticMeshInput input;
    input.rod_state = std::make_shared<RodMeshState>();
    auto& state = *input.rod_state;
    state.options.edge_based = true;
    state.radius = .2;
    input.vertices.resize(7 * centers.size(), 3);
    for (size_t i = 0; i < size_t(input.vertices.rows()); ++i)
        input.source_vertex_ids.push_back(100 + i);
    for (size_t i = 0; i < centers.size(); ++i) {
        Vector3d axis = Vector3d::UnitZ();
        if (closed)
            axis = ((centers[i] - centers[(i + centers.size() - 1) % centers.size()]).normalized() +
                    (centers[(i + 1) % centers.size()] - centers[i]).normalized())
                       .normalized();
        const auto q = Eigen::Quaterniond::FromTwoVectors(Vector3d::UnitZ(), axis) *
                       Eigen::AngleAxisd(phases[i], Vector3d::UnitZ());
        const size_t center = 7 * i;
        input.vertices.row(center) = centers[i].transpose();
        RodFrame frame;
        frame.center = center;
        frame.valid = true;
        frame.rotation = q;
        for (size_t k = 0; k < 6; ++k) {
            frame.ring.push_back(center + k + 1);
            const double angle = 2 * pi * k / 6;
            input.vertices.row(center + k + 1) =
                (centers[i] + state.radius * (q * Vector3d(std::cos(angle), std::sin(angle), 0)))
                    .transpose();
        }
        state.vertex_frame[center] = state.frames.size();
        state.frames.push_back(frame);
        state.vertex_rotations[center] = q;
        state.section_edges[center] = {center, center};
    }
    std::vector<size_t> path;
    for (size_t i = 0; i < centers.size() - (closed ? 0 : 1); ++i) {
        state.directed_input_edges.push_back({7 * i, 7 * ((i + 1) % centers.size())});
        state.input_edges.push_back(state.directed_input_edges.back());
        path.push_back(i);
    }
    state.rods.push_back(path);
    return input;
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

TEST_CASE(
    "Section mapping rounds only the terminal ring and keeps intermediate labels",
    "[rod][frame_mapping]")
{
    auto input = sections({{0, 0, 0}, {0, 0, 1.5}, {0, 0, 4}}, {0, .45, .9});
    auto& state = *input.rod_state;
    state.section_edges.erase(7); // An inter-frame transition, not a circular control.
    const auto before = state.frames;
    transport_rod_section_mapping(input);
    const auto& report = state.report["material_mapping"]["paths"][0];
    REQUIRE(report["completed"] == true);
    CHECK(report["per_frame_rounding"] == false);
    CHECK(report["DP_pairing"] == false);
    const double delta = .9 - pi / 3;
    CHECK(std::abs(report["delta_theta_radians"].get<double>() - delta) < 1e-12);
    CHECK(std::abs(report["delta_theta_radians"].get<double>()) <= pi);
    CHECK(report["terminal_degree_shift"] == 5);
    CHECK(state.frames[0].ring == before[0].ring);
    CHECK(state.frames[1].ring == before[1].ring);
    auto end = before[2].ring;
    std::rotate(end.begin(), end.begin() + 5, end.end());
    CHECK(state.frames[2].ring == end);
    CHECK(state.mapping_controls == std::set<size_t>{0, 14});
    const Eigen::Quaterniond middle(Eigen::AngleAxisd(delta * 1.5 / 4, Vector3d::UnitZ()));
    CHECK(state.frames[1].rotation.angularDistance(middle) < 1e-12);
    CHECK(state.frames[1].rotation.angularDistance(before[1].rotation) > .4);
    CHECK(report["closure_error_radians"].get<double>() < 1e-12);
}

TEST_CASE(
    "Closed section mapping carries its terminal permutation on one seam",
    "[rod][frame_mapping]")
{
    auto input = sections(
        {{0, 0, 0}, {2, 0, 0}, {2, 1, 1}, {1, 2, .3}, {-.5, 1, 1.2}},
        {0, .13, -.07, .23, -.11},
        true);
    auto& state = *input.rod_state;
    const auto before = state.frames;
    transport_rod_section_mapping(input);
    const auto& report = state.report["material_mapping"]["paths"][0];
    INFO(report.dump());
    REQUIRE(report["completed"] == true);
    REQUIRE(report["closed"] == true);
    CHECK(report["closure_error_radians"].get<double>() < 1e-12);
    CHECK(state.layer_slot_shifts.size() == 1);
    CHECK(state.frames.front().ring == before.front().ring);
    for (size_t i = 1; i < before.size(); ++i) {
        // Intermediate origins may change, but the oriented cycle must remain.
        auto cycle = before[i].ring;
        const auto zero = std::find(cycle.begin(), cycle.end(), state.frames[i].ring.front());
        REQUIRE(zero != cycle.end());
        std::rotate(cycle.begin(), zero, cycle.end());
        CHECK(state.frames[i].ring == cycle);
    }
    const size_t shift = report["terminal_degree_shift"];
    const auto [lo, hi] = rod_layer_rings(input, {28, 0});
    CHECK(lo == state.frames.back().ring);
    auto expected = before.front().ring;
    std::rotate(expected.begin(), expected.begin() + shift, expected.end());
    CHECK(hi == expected);
    const double density = report["twist_density"];
    for (const auto& row : report["frames"])
        CHECK(
            std::abs(
                row["correction_radians"].get<double>() -
                density * row["arc_length"].get<double>()) < 1e-12);
}

TEST_CASE(
    "Edge-based transition planes do not depend on ring phase",
    "[rod][frame_mapping][twist_regression]")
{
    auto input = field({{0, 0, 0}, {0, 0, 1}, {1, 0, 1}}, {.4, 1.2});
    auto& state = *input.rod_state;
    state.options.edge_based = true;
    average_rod_vertex_frames(state);
    const Vector3d expected = Vector3d(1, 0, 1).normalized();
    const Vector3d before = state.vertex_rotations.at(1) * Vector3d::UnitZ();
    CHECK((before - expected).norm() < 1e-12);
    // Changing an in-plane reference must not tilt the corner's geometric plane.
    state.edge_frames[1].rotation =
        Eigen::Quaterniond::FromTwoVectors(Vector3d::UnitZ(), Vector3d::UnitX()) *
        Eigen::AngleAxisd(.4, Vector3d::UnitZ());
    average_rod_vertex_frames(state);
    CHECK((state.vertex_rotations.at(1) * Vector3d::UnitZ() - before).norm() < 1e-12);
}

TEST_CASE(
    "Section correspondence does not depend on an arbitrary intermediate cyclic origin",
    "[rod][frame_mapping][twist_regression]")
{
    const std::vector<Vector3d> centers{{0, 0, 0}, {0, 0, 1.5}, {0, 0, 4}};
    auto reference = sections(centers, {0, .05, .1});
    auto relabeled = sections(centers, {0, .05, .1});
    auto& ring = relabeled.rod_state->frames[1].ring;
    std::rotate(ring.begin(), ring.begin() + 1, ring.end());
    transport_rod_section_mapping(reference);
    transport_rod_section_mapping(relabeled);
    for (size_t i = 0; i < reference.rod_state->frames.size(); ++i) {
        CHECK(
            reference.rod_state->frames[i].rotation.angularDistance(
                relabeled.rod_state->frames[i].rotation) < 1e-12);
        CHECK(reference.rod_state->frames[i].ring == relabeled.rod_state->frames[i].ring);
    }
}

TEST_CASE(
    "Planar section transport adds no twist to aligned endpoint rings",
    "[rod][frame_mapping][twist_regression]")
{
    auto input = sections({{0, 0, .5}, {0, 0, 1}, {.5, 0, 1}}, {.4, .4, .4});
    auto& state = *input.rod_state;
    const Eigen::Quaterniond first(Eigen::AngleAxisd(.4, Vector3d::UnitZ()));
    const Eigen::Quaterniond bend(Eigen::AngleAxisd(pi / 2, Vector3d::UnitY()));
    const Eigen::Quaterniond last = bend * first;
    const auto set_pose = [&](size_t i, const Eigen::Quaterniond& q) {
        auto& f = state.frames[i];
        f.rotation = q;
        state.vertex_rotations[f.center] = q;
        for (size_t k = 0; k < f.ring.size(); ++k) {
            const double angle = 2 * pi * k / f.ring.size();
            input.vertices.row(f.ring[k]) =
                (Vector3d(input.vertices.row(f.center)) +
                 state.radius * (q * Vector3d(std::cos(angle), std::sin(angle), 0)))
                    .transpose();
        }
    };
    set_pose(0, first);
    set_pose(1, Eigen::Quaterniond(Eigen::AngleAxisd(pi / 4, Vector3d::UnitY())) * first);
    set_pose(2, last);
    state.section_edges.erase(7);
    // Simulate the old initialization: an unrelated terminal phase tilted
    // the stored midpoint normal. Transport must derive the plane geometrically.
    const Eigen::Quaterniond unrelated = bend * Eigen::AngleAxisd(1.2, Vector3d::UnitZ());
    state.vertex_rotations[7] = first.slerp(.5, unrelated);
    transport_rod_section_mapping(input);
    const auto& report = state.report["material_mapping"]["paths"][0];
    REQUIRE(report["completed"] == true);
    CHECK(std::abs(report["delta_theta_radians"].get<double>()) < 1e-12);
    CHECK(state.frames.front().rotation.angularDistance(first) < 1e-12);
    CHECK(state.frames.back().rotation.angularDistance(last) < 1e-12);
    CHECK((state.frames[1].tangent - Vector3d(1, 0, 1).normalized()).norm() < 1e-12);
}

TEST_CASE("A single control has a well-defined untwisted pose", "[rod][frame_mapping]")
{
    auto input = sections({{0, 0, 0}, {0, 0, 1}, {0, 0, 2}}, {0, .2, .4});
    auto& state = *input.rod_state;
    state.section_edges.erase(0);
    state.section_edges.erase(14);
    const auto before = state.frames[1].rotation;
    transport_rod_section_mapping(input);
    const auto& report = state.report["material_mapping"]["paths"][0];
    REQUIRE(report["completed"] == true);
    CHECK(report["span_length"] == 0);
    CHECK(report["delta_theta_radians"] == 0);
    CHECK(state.frames[1].rotation.angularDistance(before) < 1e-12);
    CHECK(state.mapping_controls == std::set<size_t>{7});
}
