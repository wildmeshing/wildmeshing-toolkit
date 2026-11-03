#include <wmtk/utils/EnvelopeBudget.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {

void check_envelope_reference(
    const double eps,
    const double simplify_eps,
    const bool envelope_around_simplified,
    const bool simplification_ran)
{
    if (envelope_around_simplified) {
        if (simplification_ran) return;
        log_and_throw_error(
            "envelope_reference \"simplified\": no simplification runs, so there is no simplified "
            "geometry to build the envelopes around. Use envelope_reference \"input\", or turn the "
            "simplification on.");
    }
    if (!simplification_ran || simplify_eps < eps) return;
    log_and_throw_error(
        "envelope_reference \"input\": the simplification's eps {:.6} is not below the "
        "optimizer's eps {:.6}, so the simplified geometry may start outside the envelope it is "
        "held to. Lower the simplification's eps (eps_simplify_rel or eps_simplify), or build the "
        "envelopes around the simplified geometry (envelope_reference \"simplified\").",
        simplify_eps,
        eps);
}

} // namespace wmtk::utils
