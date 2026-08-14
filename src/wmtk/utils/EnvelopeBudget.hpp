#pragma once

namespace wmtk::utils {

/**
 * @brief How much of the deviation budget the optimizer's envelope may spend.
 *
 * Every app in the toolkit promises the same thing to its caller: the output stays within
 * `eps` of the ORIGINAL input. Two stages move geometry, and the promise is about their sum.
 *
 *  - The simplification moves the input by up to `simplify_eps`.
 *  - The optimizer then moves that by up to whatever its own envelope allows.
 *
 * When the optimizer's envelope is built around the *input*, the two are not additive -- the
 * envelope measures against the thing the promise is about -- and the optimizer gets the whole
 * `eps`. When it is built around the *simplified* geometry instead, they are: by the triangle
 * inequality anything within `eps - simplify_eps` of the simplification is within `eps` of the
 * input, so that remainder is what the optimizer may spend.
 *
 * Building around the simplified geometry is otherwise the better choice, which is why the
 * option exists: the mesh then starts at the CENTRE of the envelope it is judged against
 * rather than somewhere inside it, and since the envelope is a hard veto rather than a
 * penalty, a mesh handed over close to the boundary has most of its moves refused.
 *
 * @param eps                       the total tolerance, as advertised to the caller
 * @param simplify_eps              what the simplification was allowed to spend
 * @param envelope_around_simplified whether the optimizer's envelope is built around the
 *                                  simplified geometry rather than the input
 * @param simplification_ran        whether a simplification actually ran. Charging for one
 *                                  that did not run confines the optimizer to a fraction of
 *                                  the tolerance its result is judged against, for no gain:
 *                                  the geometry is the input, already centred in the full
 *                                  envelope. Which config settings imply this is per-app --
 *                                  `skip_simplify` in tetwild and triwild, and in simwild also
 *                                  `preserve_topology` outside the remeshing operation.
 *
 * @return the eps to build the optimizer's envelope with; always > 0.
 *
 * A `simplify_eps` at or above `eps` leaves nothing to spend. That is a misconfiguration, and
 * this warns and returns the full `eps` rather than failing: the alternative is worse than it
 * looks, because SampleEnvelope::init squares its argument, so a *negative* eps becomes a
 * positive eps2 and yields a silently smaller envelope instead of an error.
 */
double optimization_envelope_eps(
    double eps,
    double simplify_eps,
    bool envelope_around_simplified,
    bool simplification_ran);

} // namespace wmtk::utils
