#pragma once

namespace wmtk::utils {

/**
 * @brief Refuse an envelope_reference the run cannot honour, before any work.
 *
 * envelope_reference says which geometry every envelope, tube and anchor is built around, and
 * each one is built at its full radius (eps times its ratio) either way; what differs is what the
 * output is then within eps of.
 *
 *  - "input": the original input. The simplification moves it by up to `simplify_eps` first, so
 *    `simplify_eps` must be below `eps`, else the simplified geometry may start outside the
 *    envelope around the input it is held to, where every operation near it is vetoed.
 *  - "simplified": the simplified geometry, which replaces the input: the output is within eps of
 *    it, and so within `simplify_eps` + eps of the input. Any `simplify_eps` is allowed, a
 *    coarse simplification of a dense input included. With no simplification there is no
 *    simplified geometry, and asking for it is a misconfiguration.
 *
 * Until 2026-11 "simplified" charged the simplification against eps, building around the
 * simplified geometry at eps - `simplify_eps`; at or above eps it fell back to the full eps with
 * a warning, a switch of guarantee no caller asked for.
 *
 * @param eps                       the optimizer's tolerance
 * @param simplify_eps              the tolerance the simplification runs at
 * @param envelope_around_simplified envelope_reference "simplified"
 * @param simplification_ran        whether a simplification runs. Which config settings imply
 *                                  it is per-app: `simplify` in tetwild and triwild, and in
 *                                  simwild also `preserve_topology`, under which only the
 *                                  remeshing operation simplifies, after the insertion.
 */
void check_envelope_reference(
    double eps,
    double simplify_eps,
    bool envelope_around_simplified,
    bool simplification_ran);

} // namespace wmtk::utils
