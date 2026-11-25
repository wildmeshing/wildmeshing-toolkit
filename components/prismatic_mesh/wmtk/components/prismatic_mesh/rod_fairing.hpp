#pragma once
#include <cmath>
#include "rod_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
struct RodFairingRow
{
    std::array<size_t, 3> vertices;
    std::array<Eigen::Matrix3d, 3> coefficients;
    Vector3d constant;
    double weight = 1;
    bool longitudinal = false;
    Vector3d residual(const MatrixXd& positions) const
    {
        Vector3d value = -constant;
        for (int j = 0; j < 3; ++j)
            value += coefficients[j] * positions.row(vertices[j]).transpose();
        return value;
    }
};

inline std::vector<RodFairingRow> rod_fairing_rows(const PrismaticMeshInput& input)
{
    const auto& state = *input.rod_state;
    const double radius = state.radius;
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const auto p = [&](size_t v) -> Vector3d { return input.vertices.row(v).transpose(); };
    std::vector<RodFairingRow> rows;
    for (const auto& f : state.frames) {
        if (!f.valid || f.ring.size() < 3) continue;
        const double cosine = std::cos(2 * std::acos(-1.) / f.ring.size());
        for (size_t k = 0; k < f.ring.size(); ++k) {
            RodFairingRow row;
            row.vertices = {
                f.ring[(k + f.ring.size() - 1) % f.ring.size()],
                f.ring[k],
                f.ring[(k + 1) % f.ring.size()]};
            row.coefficients = {
                identity / radius,
                -2 * cosine * identity / radius,
                identity / radius};
            row.constant = (2 - 2 * cosine) * p(f.center) / radius;
            row.weight = .25;
            rows.push_back(row);
        }
    }
    // Transport radial vectors between section planes before comparing them.
    // This removes the centerline's actual bends from the fairing residual.
    for (const auto& path : state.rods) {
        if (path.empty()) continue;
        std::vector<size_t> centers{state.directed_input_edges[path.front()][0]};
        for (size_t e : path) centers.push_back(state.directed_input_edges[e][1]);
        if (centers.size() > 3 && centers.front() == centers.back()) {
            centers.pop_back();
            const size_t first = centers.front(), last = centers.back();
            centers.insert(centers.begin(), last);
            centers.push_back(first);
        }
        for (size_t i = 1; i + 1 < centers.size(); ++i) {
            const size_t a = centers[i - 1], b = centers[i], c = centers[i + 1];
            if (!state.vertex_frame.count(a) || !state.vertex_frame.count(b) ||
                !state.vertex_frame.count(c))
                continue;
            const auto& fa = state.frames[state.vertex_frame.at(a)];
            const auto& fb = state.frames[state.vertex_frame.at(b)];
            const auto& fc = state.frames[state.vertex_frame.at(c)];
            if (fa.ring.size() != fb.ring.size() || fc.ring.size() != fb.ring.size()) continue;
            const double la = (p(a) - p(b)).norm(), lc = (p(c) - p(b)).norm();
            if (!(la > 0 && lc > 0)) continue;
            const Vector3d axis = fb.rotation * Vector3d::UnitZ();
            const Eigen::Matrix3d ra =
                Eigen::Quaterniond::FromTwoVectors(fa.rotation * Vector3d::UnitZ(), axis)
                    .toRotationMatrix();
            const Eigen::Matrix3d rc =
                Eigen::Quaterniond::FromTwoVectors(fc.rotation * Vector3d::UnitZ(), axis)
                    .toRotationMatrix();
            const double wa = lc / (la + lc), wc = la / (la + lc);
            const auto [middle_a, ring_a] = rod_layer_rings(input, {b, a});
            const auto [middle_c, ring_c] = rod_layer_rings(input, {b, c});
            for (size_t k = 0; k < fb.ring.size(); ++k) {
                RodFairingRow row;
                row.vertices = {ring_a[k], fb.ring[k], ring_c[k]};
                row.coefficients = {-wa * ra / radius, identity / radius, -wc * rc / radius};
                row.constant = (p(b) - wa * ra * p(a) - wc * rc * p(c)) / radius;
                row.longitudinal = true;
                rows.push_back(row);
            }
        }
    }
    return rows;
}
} // namespace wmtk::components::prismatic_mesh
