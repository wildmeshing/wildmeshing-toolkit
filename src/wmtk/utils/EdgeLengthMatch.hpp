#pragma once

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace wmtk::utils {

/**
 * @brief Report how well a mesh's edge lengths match their prescribed targets.
 *
 * @param ratios  one entry per edge: its length divided by its own target length, which is
 *                `l * sizing_scalar` averaged over the endpoints -- the exact quantity the
 *                split and collapse length gates compare. 1.0 is an edge exactly at target.
 * @param tag     which mesh produced it, so a run covering both can be told apart.
 *
 * `short` / `inband` / `long` are the fractions below, inside and above [4/5, 4/3], the
 * interval the two length gates bound: the split gate refuses to split below 4/3 and the
 * collapse gate stops requiring a quality improvement below 4/5. An optimizer honouring its
 * sizing field would leave most edges inside it, so the fractions outside -- and their
 * asymmetry, since only the split side is enforced by a hard threshold -- say how far the
 * result is from the size it was asked for.
 */
inline void log_edge_length_match(std::vector<double> ratios, const std::string& tag)
{
    if (ratios.empty()) {
        logger().info("[edge-match] {} n=0", tag);
        return;
    }
    std::sort(ratios.begin(), ratios.end());
    const double n = double(ratios.size());
    const auto q = [&](double f) { return ratios[std::min(ratios.size() - 1, size_t(f * n))]; };

    double sum = 0., sumsq = 0.;
    for (const double x : ratios) {
        sum += x;
        sumsq += x * x;
    }
    const double mean = sum / n;
    const double sd = std::sqrt(std::max(0., sumsq / n - mean * mean));

    size_t below = 0, above = 0;
    for (const double x : ratios) {
        if (x < 4. / 5.) ++below;
        if (x > 4. / 3.) ++above;
    }

    logger().info(
        "[edge-match] {} n={} mean={:.4f} sd={:.4f} min={:.4f} p01={:.4f} p05={:.4f} p25={:.4f} "
        "p50={:.4f} p75={:.4f} p95={:.4f} p99={:.4f} max={:.4f} short={:.4f} inband={:.4f} "
        "long={:.4f}",
        tag,
        ratios.size(),
        mean,
        sd,
        ratios.front(),
        q(0.01),
        q(0.05),
        q(0.25),
        q(0.50),
        q(0.75),
        q(0.95),
        q(0.99),
        ratios.back(),
        double(below) / n,
        (n - double(below) - double(above)) / n,
        double(above) / n);
}

} // namespace wmtk::utils
