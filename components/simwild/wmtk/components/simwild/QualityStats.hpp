#pragma once

#include <wmtk/utils/Logger.hpp>

#include "expression_parser/Expression.hpp"

#include <cmath>
#include <string>
#include <tuple>
#include <vector>

namespace wmtk::components::simwild {

/**
 * @brief Quality of the cells governed by one entry of the quality field.
 *
 * `target_quality` resolves a cell's target by scanning the quality field and keeping the LAST
 * expression that matches, so a cell belongs to exactly one group: the last matching entry, or
 * the default group when none match. The groups therefore partition the mesh, and each group's
 * @ref target is the one the optimizer actually applies to those cells.
 */
struct QualityGroup
{
    /// The expression as written in the spec, or "(default)" for the fallback group.
    std::string label;
    /// The target the optimizer applies to this group -- the entry's value, or `stop_energy`.
    double target = 0.;
    /// Largest and summed ABSOLUTE quality; @ref avg_quality divides the sum by @ref count.
    double max_quality = -1.;
    double sum_quality = 0.;
    size_t count = 0;

    double avg_quality() const { return count > 0 ? sum_quality / count : 0.; }
    /// Quality relative to this group's own target -- the number the stopping criterion sees.
    double max_relative() const { return count > 0 ? max_quality / target : -1.; }
};

/// Per-group breakdown plus the mesh-wide relative figures `optimization_quality_stats` returns.
struct QualityBreakdown
{
    /// Default group first, then the quality field in spec order. Empty groups are kept so the
    /// output shows that an expression matched nothing rather than silently omitting it.
    std::vector<QualityGroup> groups;
    /// Mesh-wide max/avg of quality RELATIVE to each cell's own target.
    double max_relative = -1.;
    double avg_relative = 0.;
    size_t count = 0;
};

/**
 * @brief Collect the mesh-wide and per-group quality figures in a single sweep.
 *
 * Both figures need each cell's target, and resolving that means walking the quality field, so
 * they are gathered together rather than in two passes: the mesh-wide numbers alone would
 * repeat the expression evaluation that the breakdown already does.
 *
 * @param capacity      Cell capacity to scan.
 * @param quality_field The (expression, target) entries, in spec order.
 * @param base_target   Target for cells no expression matches (`stop_energy`).
 * @param is_valid      `bool(size_t)` -- whether this slot holds a live cell.
 * @param tags_of       `const CellTag&(size_t)` -- the cell's tags.
 * @param quality_of    `double(size_t)` -- the cell's ABSOLUTE quality, in the same units as
 *                      the targets (SimWildMesh passes the cube root of its stored AMIPS^3).
 */
template <class ValidFn, class TagsFn, class QualityFn>
QualityBreakdown collect_quality_breakdown(
    const size_t capacity,
    const std::vector<std::tuple<expression_parser::ExpressionPtr, double>>& quality_field,
    const double base_target,
    ValidFn&& is_valid,
    TagsFn&& tags_of,
    QualityFn&& quality_of)
{
    QualityBreakdown out;
    out.groups.resize(quality_field.size() + 1);
    out.groups[0].label = "(default)";
    out.groups[0].target = base_target;
    for (size_t i = 0; i < quality_field.size(); ++i) {
        const auto& [expr, target] = quality_field[i];
        out.groups[i + 1].label = expr->to_string();
        out.groups[i + 1].target = target;
    }

    for (size_t cid = 0; cid < capacity; ++cid) {
        if (!is_valid(cid)) {
            continue;
        }
        const auto& tags = tags_of(cid);

        // Last match wins, mirroring target_quality. Scanning backwards finds it first, and the
        // common case -- a cell matching nothing -- still costs the whole field either way.
        size_t group = 0;
        for (size_t i = quality_field.size(); i-- > 0;) {
            if (std::get<0>(quality_field[i])->eval(tags)) {
                group = i + 1;
                break;
            }
        }

        const double quality = quality_of(cid);
        QualityGroup& g = out.groups[group];
        g.max_quality = std::max(g.max_quality, quality);
        g.sum_quality += quality;
        ++g.count;

        const double relative = quality / g.target;
        out.max_relative = std::max(out.max_relative, relative);
        out.avg_relative += relative;
        ++out.count;
    }

    if (out.count > 0) {
        out.avg_relative /= out.count;
    }
    return out;
}

/**
 * @brief Log the per-group breakdown, worst group first.
 *
 * Sorted by relative quality because that is what ranks the groups against each other: a group
 * with a loose target and a high absolute quality can be converged while a stricter one is not.
 * The relative figure is also what the stopping criterion compares, so the first line is the
 * group holding the optimization back.
 */
inline void log_quality_breakdown(const QualityBreakdown& breakdown)
{
    std::vector<const QualityGroup*> order;
    order.reserve(breakdown.groups.size());
    for (const auto& g : breakdown.groups) {
        order.push_back(&g);
    }
    std::sort(order.begin(), order.end(), [](const QualityGroup* a, const QualityGroup* b) {
        return a->max_relative() > b->max_relative();
    });

    logger().info(
        "Quality by tag ({} cells, max rel = {:.4}, avg rel = {:.4}):",
        breakdown.count,
        breakdown.max_relative,
        breakdown.avg_relative);
    for (const QualityGroup* g : order) {
        if (g->count == 0) {
            logger().info("  {:<20} no cells", g->label);
            continue;
        }
        logger().info(
            "  {:<20} n = {:>8}, max = {:>8.6}, avg = {:>8.6}, target = {:>6.6}, max rel = {:6>.4}",
            g->label,
            g->count,
            g->max_quality,
            g->avg_quality(),
            g->target,
            g->max_relative());
    }
}

} // namespace wmtk::components::simwild
