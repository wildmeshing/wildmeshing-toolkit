#pragma once
#include "mixed_reference.hpp"

namespace wmtk::components::prismatic_mesh::quality {

inline native::Rat energy_contribution(double value)
{
    if (!std::isfinite(value)) throw std::runtime_error("nonfinite_combined_energy");
    return native::Rat(value);
}

// Built from the current mesh before each smoothing stage/checkpoint. It owns
// no connectivity or reference pointers that survive a topology transaction.
struct ThicknessEnergy
{
    struct Pair
    {
        size_t a, b;
        double weight;
    };
    const MixedReference& reference;
    double point_weight, smoothing_weight, tolerance;
    std::vector<uint8_t> referenced, interface_vertex;
    std::vector<Pair> pairs;
    std::vector<std::vector<size_t>> incident, anchored;
    size_t feature_edges = 0;

    ThicknessEnergy(
        const Mesh& m,
        const MixedReference& ref,
        double point_weight,
        double smoothing_weight = 10,
        double tolerance = .05,
        double feature_angle = 30)
        : reference(ref)
        , point_weight(point_weight)
        , smoothing_weight(smoothing_weight)
        , tolerance(tolerance)
        , referenced(m.p.size(), false)
        , interface_vertex(m.p.size(), false)
        , incident(m.p.size())
        , anchored(m.p.size())
    {
        for (size_t v = 0; v < m.p.size(); ++v)
            if (!m.star[v].empty() && ref.offset(m, v)) {
                referenced[v] = true;
                if (m.correspondence[v] >= 0) anchored.at(m.correspondence[v]).push_back(v);
            }
        std::set<Edge> candidates;
        for (const auto& [key, f] : faces_of(m.cells)) {
            const bool shell = m.cells[f.a].region == 2;
            const bool boundary = f.b == size_t(-1);
            const bool material = !boundary && m.cells[f.a].region != m.cells[f.b].region &&
                                  (shell || m.cells[f.b].region == 2);
            if (!(material || (shell && boundary))) continue;
            for (size_t v : f.v) interface_vertex[v] = true;
            for (size_t j = 0; j < f.v.size(); ++j)
                candidates.insert(native::edge(f.v[j], f.v[(j + 1) % f.v.size()]));
        }
        const double cosine = std::cos(feature_angle * std::acos(-1.) / 180);
        std::vector<Edge> accepted;
        std::vector<size_t> degree(m.p.size());
        for (const auto& edge : candidates) {
            const auto [a, b] = edge;
            if (!referenced[a] || !referenced[b]) continue;
            // Do not couple opposite sheets, reference-normal creases or
            // interpolated interior-height transitions to a full-height front.
            if (ref.normal[a].dot(ref.normal[b]) < cosine ||
                std::max(ref.height[a], ref.height[b]) >
                    1.5 * std::min(ref.height[a], ref.height[b])) {
                ++feature_edges;
                continue;
            }
            accepted.push_back(edge);
            ++degree[a];
            ++degree[b];
        }
        for (const auto& [a, b] : accepted) {
            const size_t i = pairs.size();
            pairs.push_back({a, b, 2. / double(degree[a] + degree[b])});
            incident[a].push_back(i);
            incident[b].push_back(i);
        }
    }

    bool unreferenced_interface(const Mesh& m, size_t v) const
    {
        return (point_weight > 0 || smoothing_weight > 0) && interface_vertex[v] && !m.input[v] &&
               !m.outer[v] && !referenced[v];
    }

    // All points in group translate by the same displacement. Include both
    // moved offsets and offsets whose discrete input anchors move, even when
    // the latter are outside group. A parent/child rigid translation has zero
    // height derivative, while a moved anchor alone has the opposite sign.
    double local(const Mesh& m, const std::vector<size_t>& group, V* gradient = nullptr) const
    {
        const std::set<size_t> members(group.begin(), group.end());
        std::set<size_t> affected, edges;
        for (size_t v : group) {
            if (referenced[v]) affected.insert(v);
            affected.insert(anchored[v].begin(), anchored[v].end());
        }
        for (size_t v : affected) edges.insert(incident[v].begin(), incident[v].end());
        const auto velocity = [&](size_t v) -> V {
            const bool point = members.count(v) != 0;
            const bool anchor =
                m.correspondence[v] >= 0 && members.count(size_t(m.correspondence[v])) != 0;
            return (int(point) - int(anchor)) * reference.normal[v] / reference.height[v];
        };
        double energy = 0;
        if (point_weight > 0)
            for (size_t v : affected) {
                const double error = reference.height_error(m, v);
                energy += point_weight * error * error;
                if (gradient) *gradient += 2 * point_weight * error * velocity(v);
            }
        if (smoothing_weight > 0)
            for (size_t i : edges) {
                const auto& p = pairs[i];
                const double error =
                    reference.height_error(m, p.a) - reference.height_error(m, p.b);
                const double weight = smoothing_weight * p.weight;
                energy += weight * error * error;
                if (gradient) *gradient += 2 * weight * error * (velocity(p.a) - velocity(p.b));
            }
        return energy;
    }

    size_t activate(const Mesh& m, std::vector<double>& priority) const
    {
        size_t violations = 0;
        const auto mark = [&](size_t v, double error) {
            priority[v] = std::max(priority[v], 20. + error * error);
        };
        if (point_weight > 0)
            for (size_t v = 0; v < referenced.size(); ++v)
                if (referenced[v]) {
                    const double error = std::abs(reference.height_error(m, v));
                    if (error <= tolerance) continue;
                    ++violations;
                    mark(v, error);
                }
        if (smoothing_weight > 0)
            for (const auto& p : pairs) {
                const double error =
                    std::abs(reference.height_error(m, p.a) - reference.height_error(m, p.b));
                if (error <= tolerance) continue;
                ++violations;
                mark(p.a, error);
                mark(p.b, error);
            }
        return violations;
    }

    struct Score
    {
        native::Rat energy;
        Json report;
        bool target_met = true;
    };
    Score score(const Mesh& m) const
    {
        Score result;
        native::Rat point_energy, pair_energy;
        long double absolute_error = 0;
        size_t count = 0, bad_points = 0, bad_pairs = 0, missing = 0;
        double max_error = 0, max_jump = 0;
        for (size_t v = 0; v < referenced.size(); ++v) {
            missing += unreferenced_interface(m, v);
            if (!referenced[v]) continue;
            ++count;
            const double error = reference.height_error(m, v);
            absolute_error += std::abs(error);
            max_error = std::max(max_error, std::abs(error));
            bad_points += std::abs(error) > tolerance;
            if (point_weight > 0)
                point_energy = point_energy + energy_contribution(point_weight * error * error);
        }
        for (const auto& p : pairs) {
            const double error = reference.height_error(m, p.a) - reference.height_error(m, p.b);
            max_jump = std::max(max_jump, std::abs(error));
            bad_pairs += std::abs(error) > tolerance;
            if (smoothing_weight > 0)
                pair_energy =
                    pair_energy + energy_contribution(smoothing_weight * p.weight * error * error);
        }
        result.energy = point_energy + pair_energy;
        result.target_met =
            (point_weight == 0 || bad_points == 0) && (smoothing_weight == 0 || bad_pairs == 0);
        result.report = {
            {"referenced_vertices", count},
            {"relative_tolerance", tolerance},
            {"points_over_tolerance", bad_points},
            {"pairs_over_tolerance", bad_pairs},
            {"mean_absolute_height_error", double(absolute_error / std::max(size_t(1), count))},
            {"maximum_absolute_height_error", max_error},
            {"maximum_neighbor_height_jump", max_jump},
            {"neighbor_pairs", pairs.size()},
            {"feature_edges_excluded", feature_edges},
            {"unreferenced_interface_vertices", missing},
            {"unreferenced_interface_vertices_fixed", 0},
            {"point_energy", point_energy.to_double()},
            {"neighbor_energy", pair_energy.to_double()},
            {"target_met", result.target_met}};
        return result;
    }
};
} // namespace wmtk::components::prismatic_mesh::quality
