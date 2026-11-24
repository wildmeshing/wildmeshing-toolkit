#pragma once
#include <Eigen/SVD>
#include "thickness_energy.hpp"
namespace wmtk::components::prismatic_mesh::quality {
inline double mixed_loss(double q, double target, double* derivative = nullptr)
{
    const double z = std::max(0., q / target - 1);
    if (derivative) *derivative = 2 * z / target + 1e-8 / q;
    return z * z + 1e-8 * std::log(q);
}
inline Json mixed_score(const Mesh& m, double target)
{
    size_t bad = 0;
    double maximum = 0;
    long double loss = 0;
    for (const auto& c : m.cells) {
        bad += c.energy > target;
        maximum = std::max(maximum, c.energy);
        loss += mixed_loss(c.energy, target);
    }
    return {
        {"maximum_AMIPS", maximum},
        {"over_target", bad},
        {"target", target},
        {"tail_energy", double(loss)}};
}
struct MixedEnergyScore
{
    native::Rat value;
    ThicknessEnergy::Score thickness;
    Json report;
};
inline MixedEnergyScore mixed_energy_score(
    const Mesh& m,
    const MixedReference& reference,
    double target,
    double thickness_weight,
    double thickness_smoothing_weight,
    double thickness_tolerance,
    double thickness_feature_angle)
{
    const ThicknessEnergy energy(
        m,
        reference,
        thickness_weight,
        thickness_smoothing_weight,
        thickness_tolerance,
        thickness_feature_angle);
    MixedEnergyScore score;
    score.thickness = energy.score(m);
    native::Rat amips;
    // A constant extreme tet must not hide a small thickness improvement at
    // unrelated vertices. Sum the floating-point contributions exactly for
    // checkpoint comparisons; the local derivatives remain floating point.
    for (const auto& c : m.cells) amips = amips + energy_contribution(mixed_loss(c.energy, target));
    score.value = amips + score.thickness.energy;
    score.report = {
        {"total", score.value.to_double()},
        {"AMIPS", amips.to_double()},
        {"point_thickness", score.thickness.report["point_energy"]},
        {"neighbor_thickness", score.thickness.report["neighbor_energy"]}};
    return score;
}
inline std::vector<V> cell_gradient(const Mesh& m, const Cell& c)
{
    std::vector<V> grad(c.v.size(), V::Zero());
    if (c.type == 4) {
        std::array<V, 4> x, dg;
        for (int j = 0; j < 4; ++j) x[j] = m.p[c.v[j]] - m.p[c.v[0]];
        const double det = native::determinant(m.p, c.v);
        double s = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < i; ++j) s += (x[i] - x[j]).squaredNorm();
        dg[1] = x[2].cross(x[3]);
        dg[2] = x[3].cross(x[1]);
        dg[3] = x[1].cross(x[2]);
        dg[0] = -dg[1] - dg[2] - dg[3];
        const V sum = x[0] + x[1] + x[2] + x[3];
        for (int j = 0; j < 4; ++j)
            grad[j] = c.energy * ((8 * x[j] - 2 * sum) / s - (2. / 3) * dg[j] / det);
        return grad;
    }
    for (const auto& q : native::quadrature(c.type, 4)) {
        const M a = native::matrix(m.p, c, q);
        const double det = a.determinant(), s = a.squaredNorm();
        M cof;
        cof.col(0) = a.col(1).cross(a.col(2));
        cof.col(1) = a.col(2).cross(a.col(0));
        cof.col(2) = a.col(0).cross(a.col(1));
        const M g = q.w / std::pow(det, 2. / 3) * (2 * a - (2. / 3) * (s / det) * cof);
        for (size_t j = 0; j < c.v.size(); ++j) grad[j] += g * q.g[j];
    }
    return grad;
}
struct InputFreedom
{
    std::vector<M> tangent;
    std::vector<std::vector<InputFace>> faces;
    std::vector<std::vector<size_t>> children;
    explicit InputFreedom(const Mesh& m, const Rules& rules)
        : tangent(m.p.size(), M::Identity())
        , faces(m.p.size())
        , children(m.p.size())
    {
        for (const auto& [k, f] : rules.kept)
            for (size_t v : f.v) faces[v].push_back(f);
        for (size_t v = 0; v < m.p.size(); ++v) {
            if (m.outer[v]) tangent[v].setZero();
            if (!m.input[v]) {
                if (m.correspondence[v] >= 0 && m.input.at(m.correspondence[v]))
                    children[m.correspondence[v]].push_back(v);
                continue;
            }
            if (faces[v].empty()) {
                tangent[v].setZero();
                continue;
            }
            Eigen::MatrixXd normals(faces[v].size(), 3);
            for (size_t j = 0; j < faces[v].size(); ++j)
                normals.row(j) = rules.patch_normal[faces[v][j].patch].transpose();
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(normals, Eigen::ComputeFullV);
            M projection = M::Identity();
            for (int j = 0; j < svd.singularValues().size(); ++j)
                if (svd.singularValues()[j] > 1e-10)
                    projection -= svd.matrixV().col(j) * svd.matrixV().col(j).transpose();
            tangent[v] = projection;
        }
    }
};
inline bool mixed_vertex_step(
    Mesh& m,
    const Rules& rules,
    MixedReference& reference,
    const InputFreedom& freedom,
    size_t v,
    double target,
    double thickness_weight,
    Json* detail = nullptr,
    const ThicknessEnergy* thickness = nullptr)
{
    if (m.outer[v] || m.star[v].empty() || freedom.tangent[v].squaredNorm() < .1) return false;
    if (thickness && thickness->unreferenced_interface(m, v)) return false;
    std::vector<size_t> group{v};
    if (m.input[v])
        for (size_t child : freedom.children[v]) {
            if (m.outer[child]) return false;
            if (thickness && thickness->unreferenced_interface(m, child)) return false;
            group.push_back(child);
        }
    std::set<size_t> patch;
    for (size_t u : group) patch.insert(m.star[u].begin(), m.star[u].end());
    std::set<size_t> members(group.begin(), group.end());
    V gradient = V::Zero();
    double before = 0;
    for (size_t ci : patch) {
        const auto& c = m.cells[ci];
        double derivative;
        before += mixed_loss(c.energy, target, &derivative);
        if (derivative == 0) continue;
        const auto g = cell_gradient(m, c);
        for (size_t j = 0; j < c.v.size(); ++j)
            if (members.count(c.v[j])) gradient += derivative * g[j];
    }
    const bool constrained = reference.offset(m, v);
    if (thickness)
        before += thickness->local(m, group, &gradient);
    else if (constrained) {
        const double e = reference.height_error(m, v);
        before += thickness_weight * e * e;
        gradient += 2 * thickness_weight * e / reference.height[v] * reference.normal[v];
    }
    const double unconstrained_norm = gradient.cwiseAbs().maxCoeff();
    gradient = freedom.tangent[v] * gradient;
    // Active input patches can have holes where input faces were released into
    // tet components. Preserve their boundary footprint without freezing every
    // interior point; area is affine in a single moved vertex.
    std::map<size_t, V> area_gradient;
    std::vector<V> area_basis;
    if (m.input[v]) {
        for (const auto& f : freedom.faces[v]) {
            const int j = std::find(f.v.begin(), f.v.end(), v) - f.v.begin();
            const V g =
                .5 *
                (m.p[f.v[(j + 1) % 3]] - m.p[f.v[(j + 2) % 3]]).cross(rules.patch_normal[f.patch]);
            auto it = area_gradient.find(f.patch);
            if (it == area_gradient.end())
                area_gradient.emplace(f.patch, g);
            else
                it->second += g;
        }
        for (const auto& [patch_id, g] : area_gradient) {
            V n = freedom.tangent[v] * g;
            for (const auto& basis : area_basis) n -= n.dot(basis) * basis;
            if (n.norm() > 1e-11) {
                n.normalize();
                area_basis.push_back(n);
                gradient -= gradient.dot(n) * n;
            }
        }
    }
    if (!gradient.allFinite() || !(gradient.cwiseAbs().maxCoeff() > 1e-10 * unconstrained_norm))
        return false;
    std::map<size_t, double> old_area;
    if (m.input[v])
        for (const auto& face : freedom.faces[v]) old_area[face.patch] += rules.area(m, face);
    std::vector<double> lengths;
    for (size_t ci : patch)
        for (size_t u : m.cells[ci].v)
            if (!members.count(u)) {
                double l = (m.p[u] - m.p[v]).norm();
                if (l > 1e-12) lengths.push_back(l);
            }
    if (lengths.empty()) return false;
    std::nth_element(lengths.begin(), lengths.begin() + lengths.size() / 2, lengths.end());
    const double step = .2 * std::min(
                                 lengths[lengths.size() / 2],
                                 std::max(2 * reference.h, reference.base_length * m.sizing[v]));
    const V direction = -step * (gradient / gradient.cwiseAbs().maxCoeff()).normalized();
    std::vector<V> old;
    for (size_t u : group) old.push_back(m.p[u]);
    std::vector<Cell> next;
    next.reserve(patch.size());
    for (int bt = 0; bt < 44; ++bt) {
        const V candidate = reference.project(m, v, old[0] + std::ldexp(1., -bt) * direction),
                delta = candidate - old[0];
        const double slope = gradient.dot(delta);
        if (!(slope < 0)) continue;
        for (size_t j = 0; j < group.size(); ++j) m.p[group[j]] = old[j] + delta;
        bool valid = true;
        std::map<size_t, double> new_area;
        if (m.input[v])
            for (const auto& face : freedom.faces[v]) {
                new_area[face.patch] += rules.area(m, face);
                if (!(rules.area(m, face) > 0) ||
                    std::abs(rules.patch_normal[face.patch].dot(
                        m.p[v] - rules.patch_origin[face.patch])) > 1e-10) {
                    valid = false;
                    break;
                }
            }
        if (valid)
            for (const auto& [patch_id, a] : old_area)
                if (std::abs(new_area[patch_id] - a) >
                    std::max(1e-16, 1e-10 * rules.patch_area[patch_id])) {
                    valid = false;
                    break;
                }
        double after = 0;
        next.clear();
        if (valid)
            for (size_t ci : patch) {
                auto c = m.cells[ci];
                if (!std::isfinite(native::quality(m.p, c))) {
                    valid = false;
                    break;
                }
                after += mixed_loss(c.energy, target);
                next.push_back(std::move(c));
            }
        if (thickness)
            after += thickness->local(m, group);
        else if (constrained) {
            const double e = reference.height_error(m, v);
            after += thickness_weight * e * e;
        }
        if (valid && after < before && after <= before + 1e-4 * slope) {
            size_t j = 0;
            for (size_t ci : patch) m.cells[ci] = std::move(next[j++]);
            if (m.input[v]) {
                m.base[v] = m.initial[v] = m.p[v];
                for (size_t child : freedom.children[v]) {
                    m.base[child] += delta;
                    m.initial[child] += delta;
                }
            }
            if (detail)
                *detail = {
                    {"vertex", v},
                    {"coupled_vertices", group.size()},
                    {"before", before},
                    {"after", after},
                    {"backtracks", bt}};
            return true;
        }
        for (size_t j = 0; j < group.size(); ++j) m.p[group[j]] = old[j];
    }
    for (size_t j = 0; j < group.size(); ++j) m.p[group[j]] = old[j];
    return false;
}
inline Json smooth_mixed(
    Mesh& m,
    const Rules& rules,
    MixedReference& reference,
    int sweeps,
    double optimize_target,
    double stop_target,
    double thickness_weight,
    Deadline& deadline,
    double thickness_smoothing_weight = 10,
    double thickness_tolerance = .05,
    double thickness_feature_angle = 30)
{
    reference.extend(m);
    InputFreedom freedom(m, rules);
    const ThicknessEnergy thickness(
        m,
        reference,
        thickness_weight,
        thickness_smoothing_weight,
        thickness_tolerance,
        thickness_feature_angle);
    for (size_t v = 0; v < m.p.size(); ++v)
        if (thickness.unreferenced_interface(m, v)) freedom.tangent[v].setZero();
    const auto before_thickness = thickness.score(m);
    Json history = Json::array();
    size_t total = 0, input_moves = 0, native_moves = 0;
    std::vector<uint8_t> native_vertex(m.p.size(), false);
    for (const auto& c : m.cells)
        if (c.type != 4)
            for (size_t v : c.v) native_vertex[v] = true;
    for (int pass = 0; pass < sweeps && !deadline.expired(); ++pass) {
        std::vector<double> priority(m.p.size());
        size_t bad = 0;
        for (const auto& c : m.cells)
            if (c.energy > stop_target) {
                ++bad;
                for (size_t v : c.v) {
                    priority[v] = std::max(priority[v], c.energy);
                    if (m.correspondence[v] >= 0)
                        priority[m.correspondence[v]] =
                            std::max(priority[m.correspondence[v]], c.energy);
                }
            }
        const size_t thickness_bad = thickness.activate(m, priority);
        if (!bad && !thickness_bad) break;
        std::vector<size_t> active;
        for (size_t v = 0; v < m.p.size(); ++v)
            if (priority[v] > 0 && !m.outer[v] && freedom.tangent[v].squaredNorm() > .1)
                active.push_back(v);
        std::sort(active.begin(), active.end(), [&](size_t a, size_t b) {
            return priority[a] != priority[b] ? priority[a] > priority[b] : a < b;
        });
        size_t accepted = 0;
        for (size_t v : active) {
            if (deadline.expired()) break;
            if (mixed_vertex_step(
                    m,
                    rules,
                    reference,
                    freedom,
                    v,
                    optimize_target,
                    thickness_weight,
                    nullptr,
                    &thickness)) {
                ++accepted;
                input_moves += bool(m.input[v]);
                native_moves += native_vertex[v];
            }
        }
        total += accepted;
        const auto score = mixed_score(m, stop_target);
        const auto height_score = thickness.score(m);
        history.push_back(
            {{"sweep", pass + 1},
             {"active", active.size()},
             {"accepted", accepted},
             {"all_mixed", score},
             {"thickness", height_score.report},
             {"seconds", deadline.seconds()}});
        logger().debug("Mixed smoothing: {}", history.back().dump());
        if (!accepted || (score["over_target"] == 0 && height_score.target_met)) break;
    }
    return {
        {"method",
         "native tet/prism/pyramid AMIPS gradients; shared-vertex and input-column constrained "
         "relocation"},
        {"accepted", total},
        {"input_column_moves", input_moves},
        {"native_corner_moves", native_moves},
        {"optimize_target", optimize_target},
        {"stop_target", stop_target},
        {"initial_thickness", before_thickness.report},
        {"final_thickness", thickness.score(m).report},
        {"history", history}};
}
} // namespace wmtk::components::prismatic_mesh::quality
