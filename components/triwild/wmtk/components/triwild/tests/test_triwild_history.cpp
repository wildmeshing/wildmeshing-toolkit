// triwild's curve history under its options, on the memory route: what recording changes, and
// that the history it returns ends at the mesh it returns.

#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/triwild/triwild.hpp>
#include <wmtk/history/History.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

using namespace wmtk;
using namespace wmtk::components;

namespace {

/// A polyline through `v` as an edge mesh, closed or open.
MeshData polyline(const std::vector<Eigen::Vector3d>& v, const bool closed)
{
    MeshData m;
    m.V.resize(Eigen::Index(v.size()), 3);
    for (size_t i = 0; i < v.size(); ++i) m.V.row(Eigen::Index(i)) = v[i].transpose();
    const size_t ne = closed ? v.size() : v.size() - 1;
    auto& E = m.cells[CellType::Edge].connectivity;
    E.resize(Eigen::Index(ne), 2);
    for (size_t i = 0; i < ne; ++i)
        E.row(Eigen::Index(i)) << int64_t(i), int64_t((i + 1) % v.size());
    return m;
}

std::vector<Eigen::Vector3d> circle(const int n, const double r = 1, const double cx = 0)
{
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i < n; ++i) {
        const double a = 2 * M_PI * i / n;
        v.push_back(Eigen::Vector3d(cx + r * std::cos(a), r * std::sin(a), 0));
    }
    return v;
}

std::vector<Eigen::Vector3d> wave(const int m)
{
    std::vector<Eigen::Vector3d> v;
    for (int i = 0; i <= m; ++i) {
        const double x = -1.6 + 3.2 * i / m;
        v.push_back(Eigen::Vector3d(x, 0.3 * std::sin(6 * x) + 0.1, 0));
    }
    return v;
}

/// The worked example's curves: a circle and an open wave crossing it twice.
MeshMap example()
{
    return {{"circle", polyline(circle(300), true)}, {"wave", polyline(wave(100), false)}};
}

/// triwild on the example, recording unless `record` is false, with `extra` options.
ComponentResult run(const nlohmann::json& extra, const bool record = true)
{
    nlohmann::json options = {
        {"input", {"circle", "wave"}},
        {"length_rel", 0.02},
        {"max_iterations", 10},
        {"num_threads", 1}};
    if (record) options["history_output"] = "record";
    if (!extra.is_null()) options.update(extra);
    return triwild::triwild(options, example());
}

history::CurveHistory history_of(const ComponentResult& r)
{
    return history::CurveHistory::from_bytes(r.second.at("history").get_binary());
}

/// The parameters the history recorded.
nlohmann::json parameters_of(const ComponentResult& r)
{
    return nlohmann::json::parse(history_of(r).metadata);
}

} // namespace

TEST_CASE("triwild-history-anchors-junctions", "[triwild][tracking]")
{
    // Recording anchors junctions -- allow_junction_cleanup defaults to false -- unless the
    // config gives allow_junction_cleanup, or preserve_feature_points is off: that anchors
    // nothing, junctions or ends, so recording leaves the option and the output alone.
    CHECK_FALSE(parameters_of(run({})).at("allow_junction_cleanup").get<bool>());
    CHECK(parameters_of(run({{"allow_junction_cleanup", true}})).at("allow_junction_cleanup"));
    const nlohmann::json no_features = {{"preserve_feature_points", false}};
    const ComponentResult recorded = run(no_features);
    CHECK(parameters_of(recorded).at("allow_junction_cleanup"));
    const ComponentResult plain = run(no_features, false);
    CHECK_FALSE(plain.second.contains("history"));
    const MeshData& a = recorded.first.at("mesh");
    const MeshData& b = plain.first.at("mesh");
    CHECK(a.V == b.V);
    CHECK(a.cell(CellType::Triangle).connectivity == b.cell(CellType::Triangle).connectivity);
}
