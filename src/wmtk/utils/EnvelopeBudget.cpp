#include <wmtk/utils/EnvelopeBudget.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {

double optimization_envelope_eps(
    const double eps,
    const double simplify_eps,
    const bool envelope_around_simplified,
    const bool simplification_ran)
{
    if (!envelope_around_simplified) {
        // The envelope measures against the input, which is what the promise is about, so
        // there is nothing to share: the optimizer gets the whole budget.
        return eps;
    }
    if (!simplification_ran) {
        // The "simplified" geometry IS the input. Nothing has been spent.
        return eps;
    }

    const double remaining = eps - simplify_eps;
    if (remaining <= 0) {
        logger().warn(
            "envelope_reference \"simplified\": the simplification's eps {:.6} leaves nothing "
            "of eps {:.6}; keeping the full eps. The output may deviate by up to {:.6} rather "
            "than {:.6}.",
            simplify_eps,
            eps,
            eps + simplify_eps,
            eps);
        return eps;
    }

    logger().info(
        "optimization envelope: eps {:.6} (eps {:.6} - the simplification's {:.6}) around the "
        "simplified geometry",
        remaining,
        eps,
        simplify_eps);
    return remaining;
}

void check_simplification_inside_envelope(
    const double eps,
    const double simplify_eps,
    const bool envelope_around_simplified,
    const bool simplification_ran)
{
    if (envelope_around_simplified || !simplification_ran || simplify_eps < eps) return;
    log_and_throw_error(
        "envelope_reference \"input\": the simplification's eps {:.6} is not below the "
        "optimizer's eps {:.6}, so the simplified geometry may start outside the envelope it is "
        "held to. Lower the simplification's eps, or build the envelopes around the simplified "
        "geometry (envelope_reference \"simplified\").",
        simplify_eps,
        eps);
}

} // namespace wmtk::utils
