#pragma once
#include <cstdint>
#include <wmtk/utils/Logger.hpp>
#include "mixed_optimizer.hpp"

namespace wmtk::components::prismatic_mesh::quality {

// Pure-background geometry cleanup. Shell/input/exterior vertices and explicit
// reservations stay fixed; topology operations are never performed here.
struct BackgroundRelocationOptions
{
    int vertex_sweeps = 80;
    int joint_sweeps = 1000;
    int neighborhood_sweeps = 1000;
    double threshold = 100;
    double time_limit = 120;
};

namespace background_detail {
inline Json
background_diagnosis(const Mesh& m, const std::vector<uint8_t>& movable, double threshold)
{
    size_t bad = 0, blocked = 0;
    double maximum = 0;
    std::map<size_t, size_t> histogram;
    Json worst;
    for (size_t i = 0; i < m.cells.size(); ++i) {
        const auto& c = m.cells[i];
        if (c.region != 3 || (c.type != 4 && c.type != 7)) continue;
        size_t free = 0;
        for (size_t v : c.v) free += movable[v];
        if (c.energy > threshold) {
            ++bad;
            ++histogram[free];
            blocked += free == 0;
        }
        if (c.energy > maximum) {
            maximum = c.energy;
            Json vertices = Json::array();
            for (size_t v : c.v)
                vertices.push_back(
                    {{"row", v},
                     {"movable", bool(movable[v])},
                     {"shell", bool(m.shell[v])},
                     {"input", bool(m.input[v])},
                     {"outer", bool(m.outer[v])},
                     {"rod_reserved", m.fixed(v)}});
            worst = {
                {"cell_row", i},
                {"type", c.type},
                {"AMIPS", c.energy},
                {"vertices", vertices}};
        }
    }
    Json counts = Json::object();
    for (auto [k, v] : histogram) counts[std::to_string(k)] = v;
    return {
        {"bad_cells", bad},
        {"maximum_AMIPS", maximum},
        {"bad_cells_with_no_movable_vertices", blocked},
        {"bad_cell_movable_vertex_histogram", counts},
        {"worst", worst}};
}


inline bool block_step(
    native::Mesh& m,
    size_t ci,
    const std::vector<uint8_t>& movable,
    double length,
    Json& diagnosis,
    bool expand,
    double target)
{
    std::vector<size_t> group;
    std::set<size_t> patch;
    for (size_t v : m.cells[ci].v)
        if (movable[v]) {
            group.push_back(v);
            patch.insert(m.star[v].begin(), m.star[v].end());
        }
    if (group.empty()) return false;
    if (expand) {
        const auto seeds = group;
        std::set<size_t> added(group.begin(), group.end());
        std::vector<std::pair<double, size_t>> neighbors;
        for (size_t v : seeds)
            for (size_t c : m.star[v])
                for (size_t u : m.cells[c].v)
                    if (movable[u] && added.insert(u).second) {
                        double priority = 0;
                        for (size_t j : m.star[u]) priority = std::max(priority, m.cells[j].energy);
                        neighbors.push_back({priority, u});
                    }
        std::sort(neighbors.begin(), neighbors.end(), [](auto a, auto b) {
            return a.first != b.first ? a.first > b.first : a.second < b.second;
        });
        for (auto [priority, u] : neighbors)
            if (group.size() < 48) group.push_back(u);
        patch.clear();
        for (size_t v : group) patch.insert(m.star[v].begin(), m.star[v].end());
    }
    std::map<size_t, size_t> index;
    for (size_t j = 0; j < group.size(); ++j) index[group[j]] = j;
    std::vector<native::V> gradient(group.size(), native::V::Zero()), old;
    long double before = 0;
    for (size_t j : patch) {
        double derivative;
        before += mixed_loss(m.cells[j].energy, target, &derivative);
        auto g = cell_gradient(m, m.cells[j]);
        for (size_t k = 0; k < m.cells[j].v.size(); ++k)
            if (auto it = index.find(m.cells[j].v[k]); it != index.end())
                gradient[it->second] += derivative * g[k];
    }
    double scale = 0;
    for (auto g : gradient) {
        if (!g.allFinite()) {
            diagnosis["nonfinite_gradient"] = true;
            return false;
        }
        scale = std::max(scale, g.cwiseAbs().maxCoeff());
    }
    if (!(scale > 0)) return false;
    for (auto& g : gradient) g /= scale;
    const auto objective_gradient = gradient;
    const Eigen::Index dimension = 3 * group.size();
    Eigen::VectorXd direction(dimension);
    for (size_t j = 0; j < group.size(); ++j) direction.segment<3>(3 * j) = -gradient[j];
    std::vector<Eigen::VectorXd> constraints;
    for (size_t j : patch) {
        const auto& c = m.cells[j];
        if (c.type != 4 || c.energy < 1e6) continue;
        native::V a = m.p[c.v[1]] - m.p[c.v[0]], b = m.p[c.v[2]] - m.p[c.v[0]],
                  d = m.p[c.v[3]] - m.p[c.v[0]];
        std::array<native::V, 4> dg;
        dg[1] = b.cross(d);
        dg[2] = d.cross(a);
        dg[3] = a.cross(b);
        dg[0] = -dg[1] - dg[2] - dg[3];
        Eigen::VectorXd normal = Eigen::VectorXd::Zero(dimension);
        for (size_t k = 0; k < 4; ++k)
            if (auto it = index.find(c.v[k]); it != index.end())
                normal.segment<3>(3 * it->second) = dg[k];
        if (normal.norm() > 0) constraints.push_back(normal.normalized());
    }
    std::vector<double> correction(constraints.size());
    for (int iteration = 0; iteration < 300; ++iteration) {
        for (size_t j = 0; j < constraints.size(); ++j) {
            direction += correction[j] * constraints[j];
            const double amount = std::max(0., -direction.dot(constraints[j]));
            direction += amount * constraints[j];
            correction[j] = -amount;
        }
    }
    double derivative = 0;
    for (size_t j = 0; j < group.size(); ++j)
        derivative += objective_gradient[j].dot(direction.segment<3>(3 * j));
    if (!(direction.norm() > 1e-15) || !(derivative < 0)) {
        diagnosis = {
            {"cone_constraints", constraints.size()},
            {"projected_norm", direction.norm()},
            {"derivative", derivative}};
        return false;
    }
    direction *= .2 * length / direction.norm();
    for (size_t j = 0; j < group.size(); ++j) gradient[j] = direction.segment<3>(3 * j);
    for (size_t v : group) old.push_back(m.p[v]);
    size_t invalid = 0, no_descent = 0, not_lower = 0;
    for (int bt = 0; bt < 48; ++bt) {
        long double slope = 0;
        for (size_t j = 0; j < group.size(); ++j) {
            m.p[group[j]] = old[j] + std::ldexp(1., -bt) * gradient[j];
            slope += (m.p[group[j]] - old[j]).dot(objective_gradient[j]);
        }
        if (!(slope < 0)) {
            ++no_descent;
            continue;
        }
        bool valid = true;
        long double after = 0;
        std::vector<native::Cell> proposed;
        for (size_t j : patch) {
            auto c = m.cells[j];
            if (!std::isfinite(native::quality(m.p, c))) {
                valid = false;
                break;
            }
            after += mixed_loss(c.energy, target);
            proposed.push_back(std::move(c));
        }
        if (!valid)
            ++invalid;
        else if (after < before) {
            size_t k = 0;
            for (size_t j : patch) m.cells[j] = std::move(proposed[k++]);
            diagnosis = {
                {"backtracks", bt},
                {"free_vertices", group.size()},
                {"before", double(before)},
                {"after", double(after)}};
            return true;
        } else
            ++not_lower;
    }
    for (size_t j = 0; j < group.size(); ++j) m.p[group[j]] = old[j];
    diagnosis = {
        {"free_vertices", group.size()},
        {"invalid_trials", invalid},
        {"unrepresentable_or_no_descent", no_descent},
        {"nondecreasing_trials", not_lower}};
    return false;
}


inline uint64_t connectivity_signature(const Mesh& m)
{
    uint64_t value = 1469598103934665603ULL;
    const auto mix = [&](uint64_t x) { value = (value ^ x) * 1099511628211ULL; };
    for (const auto& c : m.cells) {
        mix(c.type);
        mix(c.region);
        mix(c.v.size());
        for (size_t v : c.v) mix(v);
    }
    return value;
}
} // namespace background_detail

inline Json relocate_background_quality(
    Mesh& m,
    const Rules& rules,
    MixedReference& reference,
    double target,
    double thickness_weight,
    const BackgroundRelocationOptions& options = {})
{
    using namespace background_detail;
    Deadline deadline(options.time_limit);
    std::vector<uint8_t> movable(m.p.size(), false);
    for (size_t v = 0; v < m.p.size(); ++v) {
        bool background_only = !m.star[v].empty();
        for (size_t ci : m.star[v]) background_only &= m.cells[ci].region == 3;
        movable[v] = background_only && !m.input[v] && !m.outer[v] && !m.shell[v] && !m.fixed(v);
    }
    const auto initial = background_diagnosis(m, movable, options.threshold);
    const auto before_quality = quality(m);
    if (initial["bad_cells"] == 0)
        return {
            {"status", "already_below_threshold"},
            {"initial", initial},
            {"final", initial},
            {"before_quality", before_quality},
            {"after_quality", before_quality},
            {"seconds", deadline.seconds()},
            {"accepted", 0}};
    const auto frozen_positions = m.p;
    const auto topology_before = topology(m, faces_of(m.cells));
    const auto signature = connectivity_signature(m);
    const double volume0 = native::volume(m);
    InputFreedom freedom(m, rules);
    for (size_t v = 0; v < m.p.size(); ++v)
        if (!movable[v]) freedom.tangent[v].setZero();
    Json phases = Json::array();
    size_t total = 0;
    auto overall_positions = m.p;
    double overall_maximum = initial["maximum_AMIPS"];
    size_t overall_bad = initial["bad_cells"];
    std::string selected_phase = "initial";
    const auto keep_phase = [&](const char* name, const Json& state) {
        const double maximum = state["maximum_AMIPS"];
        const size_t bad = state["bad_cells"];
        if (maximum < overall_maximum || (maximum == overall_maximum && bad < overall_bad)) {
            overall_maximum = maximum;
            overall_bad = bad;
            overall_positions = m.p;
            selected_phase = name;
        }
    };
    Json history = Json::array();
    size_t vertex_total = 0;
    for (int pass = 0; pass < options.vertex_sweeps && !deadline.expired(); ++pass) {
        std::vector<double> priority(m.p.size());
        for (const auto& c : m.cells)
            if (c.region == 3 && (c.type == 4 || c.type == 7) && c.energy > options.threshold)
                for (size_t v : c.v)
                    if (movable[v]) priority[v] = std::max(priority[v], c.energy);
        std::vector<size_t> active;
        for (size_t v = 0; v < priority.size(); ++v)
            if (priority[v] > 0) active.push_back(v);
        std::sort(active.begin(), active.end(), [&](size_t a, size_t b) {
            return priority[a] != priority[b] ? priority[a] > priority[b] : a < b;
        });
        size_t accepted = 0;
        for (size_t v : active) {
            if (deadline.expired()) break;
            accepted +=
                mixed_vertex_step(m, rules, reference, freedom, v, target, thickness_weight);
        }
        vertex_total += accepted;
        const auto state = background_diagnosis(m, movable, options.threshold);
        history.push_back({{"sweep", pass + 1}, {"accepted", accepted}, {"background", state}});
        if (!accepted || state["bad_cells"] == 0) break;
    }
    auto state = background_diagnosis(m, movable, options.threshold);
    phases.push_back(
        {{"method", "vertex_gradient"},
         {"accepted", vertex_total},
         {"history", std::move(history)},
         {"final", state}});
    total += vertex_total;
    keep_phase("vertex_gradient", state);
    const auto joint_phase = [&](const char* name, int sweeps, bool expand, bool keep_best) {
        Json history = Json::array();
        size_t accepted_total = 0;
        auto best_positions = m.p;
        double best_maximum = background_diagnosis(m, movable, options.threshold)["maximum_AMIPS"];
        int selected_pass = 0;
        for (int pass = 0; pass < sweeps && !deadline.expired(); ++pass) {
            std::vector<size_t> bad;
            for (size_t ci = 0; ci < m.cells.size(); ++ci) {
                const auto& c = m.cells[ci];
                if (c.region == 3 && (c.type == 4 || c.type == 7) && c.energy > options.threshold)
                    bad.push_back(ci);
            }
            if (bad.empty()) break;
            std::sort(bad.begin(), bad.end(), [&](size_t a, size_t b) {
                return m.cells[a].energy > m.cells[b].energy;
            });
            size_t accepted = 0;
            for (size_t ci : bad) {
                if (deadline.expired()) break;
                Json detail;
                accepted +=
                    block_step(m, ci, movable, reference.base_length, detail, expand, target);
            }
            accepted_total += accepted;
            const auto state = background_diagnosis(m, movable, options.threshold);
            if (state["maximum_AMIPS"].get<double>() < best_maximum) {
                best_maximum = state["maximum_AMIPS"];
                best_positions = m.p;
                selected_pass = pass + 1;
            }
            history.push_back({{"sweep", pass + 1}, {"accepted", accepted}, {"background", state}});
            if (!accepted || state["bad_cells"] == 0) break;
        }
        if (keep_best) {
            m.p = std::move(best_positions);
            for (auto& c : m.cells) native::quality(m.p, c);
        }
        const auto state = background_diagnosis(m, movable, options.threshold);
        phases.push_back(
            {{"method", name},
             {"accepted", accepted_total},
             {"selected_pass", selected_pass},
             {"history", std::move(history)},
             {"final", state}});
        total += accepted_total;
        keep_phase(name, state);
    };
    joint_phase("volume_feasible_joint_gradient", options.joint_sweeps, false, false);
    joint_phase("one_ring_joint_gradient", options.neighborhood_sweeps, true, true);
    m.p = std::move(overall_positions);
    for (size_t v = 0; v < m.p.size(); ++v)
        if (!movable[v] && m.p[v] != frozen_positions[v])
            throw std::runtime_error("background_relocation_moved_fixed_vertex");
    if (connectivity_signature(m) != signature)
        throw std::runtime_error("background_relocation_changed_connectivity");
    for (auto& c : m.cells)
        if (!std::isfinite(native::quality(m.p, c)))
            throw std::runtime_error("background_relocation_inverted_cell");
    if (topology(m, faces_of(m.cells)) != topology_before)
        throw std::runtime_error("background_relocation_changed_topology");
    if (std::abs(native::volume(m) - volume0) > 1e-9 * std::abs(volume0))
        throw std::runtime_error("background_relocation_changed_volume");
    const auto final = background_diagnosis(m, movable, options.threshold);
    logger().info(
        "Background quality relocation: {} -> {} bad native cells; maximum {} -> {}",
        initial["bad_cells"].get<size_t>(),
        final["bad_cells"].get<size_t>(),
        initial["maximum_AMIPS"].get<double>(),
        final["maximum_AMIPS"].get<double>());
    return {
        {"status",
         final["bad_cells"] == 0 ? "complete_below_threshold" : "complete_with_residuals"},
        {"configuration",
         {{"vertex_sweeps", options.vertex_sweeps},
          {"joint_sweeps", options.joint_sweeps},
          {"neighborhood_sweeps", options.neighborhood_sweeps},
          {"threshold", options.threshold},
          {"time_limit", options.time_limit}}},
        {"initial", initial},
        {"final", final},
        {"before_quality", before_quality},
        {"after_quality", quality(m)},
        {"accepted", total},
        {"selected_phase", selected_phase},
        {"phases", std::move(phases)},
        {"timed_out", deadline.expired()},
        {"seconds", deadline.seconds()},
        {"shell_input_exterior_and_reserved_vertices_fixed", true},
        {"connectivity_preserved", true}};
}
} // namespace wmtk::components::prismatic_mesh::quality
