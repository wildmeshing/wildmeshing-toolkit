#include "OffsetPotential.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Eigenvalues>

#include <ipc/collision_mesh.hpp>
#include <ipc/esp/arbitrary_point_esp.hpp>
#include <ipc/esp/esp_parameters.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>

namespace wmtk::components::topological_offset {

namespace {
/// Unread on this path -- Phi is a point evaluation, with no quadrature in it -- but
/// ESPParameters requires an order and warns about 1.
constexpr int UNUSED_QUAD_ORDER = 2;

/// Also unread by the point evaluation (it feeds the near/far barrier split). Upstream's default.
constexpr double UNUSED_DBAR_FACTOR = 1.0;

/// The query point in the row form ipc::ArbitraryPointESP takes.
template <int DIM>
inline Eigen::RowVector<double, DIM> esp_query(const Eigen::Matrix<double, DIM, 1>& p)
{
    return p.transpose();
}
} // namespace


/**
 * @brief Everything that mentions ipc-toolkit.
 *
 * The complex never changes, so its collision mesh and ESP's broad phase are built once. An
 * evaluation builds only the small collision set around the query point, in ipc's per-thread
 * scratch, so `value`, `gradient` and `hessian` are safe to call concurrently.
 */
template <int DIM>
struct SmoothOffsetPotential<DIM>::Impl
{
    /// The complex's own vertices, compacted (see build()). Every ESP call takes the vertex
    /// configuration as an argument, and ArbitraryPointESP indexes every row as a vertex of the
    /// input.
    Eigen::MatrixXd V;
    ipc::CollisionMesh mesh;
    ipc::ESPParameters params;
    std::unique_ptr<ipc::ArbitraryPointESP<DIM>> esp;

    Impl(const double dhat)
        : params(dhat, UNUSED_DBAR_FACTOR, UNUSED_QUAD_ORDER)
    {}
};

template <int DIM>
SmoothOffsetPotential<DIM>::SmoothOffsetPotential(
    const MatrixXd& V,
    const MatrixXi& E,
    const MatrixXi& F,
    const std::vector<int>& P,
    const double delta,
    const double dhat_factor)
    : OffsetPotential<DIM>(delta, dhat_factor * delta)
{
    if (!(delta > 0.)) {
        log_and_throw_error("OffsetPotential: target_distance must be positive, got {}", delta);
    }
    if (!(dhat_factor > 1.)) {
        // At dhat_factor == 1 the offset sits exactly on the support boundary, where Phi and its
        // gradient are both zero: a vertex there gets no direction to move in and the level set
        // Phi = c does not exist. Below 1 there is no level set at all.
        log_and_throw_error(
            "OffsetPotential: offset_dhat_factor must be > 1 (the offset distance has to lie "
            "strictly inside the potential's support), got {}",
            dhat_factor);
    }

    build(V, E, F, P);

    // Calibration runs through this same code path rather than a closed form: Phi at perpendicular
    // distance delta from one large flat primitive, i.e. the level the offset takes on any flat
    // stretch of the input.
    const SmoothOffsetPotential reference(delta, dhat_factor, 0);
    VecD probe = VecD::Zero();
    probe[DIM - 1] = delta;
    m_c = reference.value(probe);
    m_grad_ref = reference.gradient(probe).norm();

    if (!(m_c > 0.) || !(m_grad_ref > 0.)) {
        log_and_throw_error(
            "OffsetPotential: calibration failed (c = {}, |grad| = {}). The flat reference "
            "produced no active contact pair, which means the collision set is not being built.",
            m_c,
            m_grad_ref);
    }

    logger().info(
        "\tSmooth offset potential ({}D): delta {:.6}, dhat {:.6} ({}x delta), level c {:.6}, "
        "|grad Phi| at the level set {:.6} | complex: {} vertices, {} segments, {} triangles, "
        "{} isolated points",
        DIM,
        m_delta,
        m_dhat,
        dhat_factor,
        m_c,
        m_grad_ref,
        V.rows(),
        E.rows(),
        F.rows(),
        P.size());
}


template <int DIM>
SmoothOffsetPotential<DIM>::SmoothOffsetPotential(const double delta, const double dhat_factor, int)
    : OffsetPotential<DIM>(delta, dhat_factor * delta)
{
    // One primitive large enough that the probe at perpendicular distance delta projects into its
    // interior and all of its boundary features lie outside the support, so exactly one pair is
    // active -- the definition of a flat stretch of input.
    const double L = 100. * m_dhat;
    if constexpr (DIM == 2) {
        MatrixXd V(2, 2);
        V << -L, 0., L, 0.;
        MatrixXi E(1, 2);
        E << 0, 1;
        build(V, E, MatrixXi(0, 3), {});
    } else {
        // (0,0) sits strictly inside this triangle at ~0.45 L from its nearest edge.
        MatrixXd V(3, 3);
        V << -L, -L, 0., L, -L, 0., 0., L, 0.;
        MatrixXi E(3, 2);
        E << 0, 1, 1, 2, 2, 0;
        MatrixXi F(1, 3);
        F << 0, 1, 2;
        build(V, E, F, {});
    }
    // m_c and m_grad_ref stay 0 here: the reference is only ever asked for value() and
    // gradient(), never for a residual.
}


template <int DIM>
SmoothOffsetPotential<DIM>::~SmoothOffsetPotential() = default;


template <int DIM>
void SmoothOffsetPotential<DIM>::build(
    const MatrixXd& V,
    const MatrixXi& E,
    const MatrixXi& F,
    const std::vector<int>& P)
{
    if (V.cols() != DIM) {
        log_and_throw_error("OffsetPotential<{}> was given {}-column vertices", DIM, V.cols());
    }
    if constexpr (DIM == 2) {
        if (F.rows() != 0) {
            log_and_throw_error("OffsetPotential<2> has no triangle primitive, got {}", F.rows());
        }
    }
    if constexpr (DIM == 3) {
        std::set<std::pair<int, int>> edges;
        for (int i = 0; i < E.rows(); ++i) {
            edges.emplace(std::min(E(i, 0), E(i, 1)), std::max(E(i, 0), E(i, 1)));
        }
        for (int f = 0; f < F.rows(); ++f) {
            for (int j = 0; j < 3; ++j) {
                const int a = F(f, j), b = F(f, (j + 1) % 3);
                if (edges.count({std::min(a, b), std::max(a, b)}) == 0) {
                    // ipc would throw the same thing from construct_faces_to_edges, but with no
                    // hint about which caller built the list.
                    log_and_throw_error(
                        "OffsetPotential<3>: edge ({}, {}) of triangle {} is missing from E. "
                        "The edge list must contain every edge of every triangle.",
                        a,
                        b,
                        f);
                }
            }
        }
    }

    // One ESP over the whole complex. ipc weighs every element so that the weights of the
    // elements containing any point of the complex sum to one (ESP supplemental, S2): a closed
    // surface or curve gets the alternating signs, the boundary of an open sheet and the ends of
    // an open curve weigh zero, and segments in no triangle and isolated points weigh one. Up to
    // ipc 3a76d751 the weights were the closed-surface signs alone: an open sheet had Phi = 0
    // beyond its boundary and a segment in no triangle entered as a negative barrier, which this
    // class patched with ipc's OGC builder, over-counting where a boundary turns back on itself.
    //
    // Compacted to the complex's own vertices: ArbitraryPointESP indexes every row it is given as
    // a vertex of the input, so a row of V in no segment, triangle or P -- another region's
    // vertex, when a per-region field passes the shared vertex list -- would enter as an isolated
    // point.
    std::vector<int> to_c(V.rows(), -1);
    std::vector<int> used;
    const auto claim = [&](const int v) {
        if (to_c[v] < 0) {
            to_c[v] = static_cast<int>(used.size());
            used.push_back(v);
        }
        return to_c[v];
    };
    MatrixXi F_c(F.rows(), 3);
    for (int f = 0; f < F.rows(); ++f) {
        for (int j = 0; j < 3; ++j) F_c(f, j) = claim(F(f, j));
    }
    MatrixXi E_c(E.rows(), 2);
    for (int i = 0; i < E.rows(); ++i) {
        E_c(i, 0) = claim(E(i, 0));
        E_c(i, 1) = claim(E(i, 1));
    }
    for (const int v : P) claim(v);

    m_impl = std::make_unique<Impl>(m_dhat);
    m_impl->V.resize(used.size(), DIM);
    for (size_t i = 0; i < used.size(); ++i) m_impl->V.row(i) = V.row(used[i]);
    m_impl->mesh = ipc::CollisionMesh(m_impl->V, E_c, F_c);
    m_impl->esp = std::make_unique<ipc::ArbitraryPointESP<DIM>>(m_impl->mesh, m_impl->params);
    // Once: the complex is fixed for this potential's lifetime.
    m_impl->esp->update(m_impl->V);
}

template <int DIM>
double SmoothOffsetPotential<DIM>::value(const VecD& p) const
{
    return (*m_impl->esp)(m_impl->V, esp_query<DIM>(p));
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::VecD SmoothOffsetPotential<DIM>::gradient(const VecD& p) const
{
    return m_impl->esp->gradient(m_impl->V, esp_query<DIM>(p));
}


template <int DIM>
typename SmoothOffsetPotential<DIM>::MatD SmoothOffsetPotential<DIM>::hessian(const VecD& p) const
{
    // The true Hessian, for the caller to project or not: the smoothing energy squares the
    // residual and takes its own Gauss-Newton approximation, which is a better-motivated route to
    // a PSD matrix than clamping this one. ESP could not be projected per term in any case,
    // because its negative weights make that invalid.
    return m_impl->esp->hessian(m_impl->V, esp_query<DIM>(p));
}


template <int DIM>
std::string SmoothOffsetPotential<DIM>::describe_active(const VecD& p) const
{
    // ArbitraryPointESP builds its collision set internally and does not hand it back, so there
    // is no per-pair breakdown; Phi, |grad Phi| and tr(H) are what a discontinuity investigation
    // compares between neighbouring samples.
    const auto [v, g, h] = m_impl->esp->evaluate(m_impl->V, esp_query<DIM>(p));
    return fmt::format("[ESP Phi={:.6g} |grad|={:.6g} tr(H)={:.6g}]", v, g.norm(), h.trace());
}


template <int DIM>
void SmoothOffsetPotential<DIM>::value_gradient(const VecD& p, double& v, VecD& g) const
{
    // value() and gradient() from ONE collision build: the build, not the per-pair barrier
    // evaluation, dominates an evaluation's cost (ipc's ArbitraryPointESP::evaluate() says so;
    // measured on the cube, value 1.07 us, gradient 1.11 us, value+gradient+Hessian 1.06 us), and
    // level_set_distance() needs both at its start.
    const auto [ev, eg, eh] = m_impl->esp->evaluate(m_impl->V, esp_query<DIM>(p));
    v = ev;
    g = eg;
}

template <int DIM>
bool SmoothOffsetPotential<DIM>::level_set_distance(const VecD& p, double& t) const
{
    // THE INVARIANT: the residual is the distance from p to the level set Phi = c measured along
    // the field -- the root of Phi(p + t n) = c, n = grad Phi(p) / |grad Phi(p)| -- with no
    // calibration constant in it. Where one pair is active Phi = b(d) and n points straight at
    // the foot point, so Phi(p + t n) = b(d - t) and the root is exactly t = d - delta, in every
    // region: on the cube (ESP over a closed convex surface, one net pair everywhere) the probe
    // reads residual / |d - delta| = 1 to 1e-9 on the flat sides, the rounded edges and the
    // corners alike, from d = 0.5 to 1.9 delta.
    //
    // What it replaces, |Phi - c| / g_ref with g_ref the calibration slope, was exact only to
    // first order (1.0135 one bar of 1e-2 delta inside the level set, 0.9867 one bar outside,
    // 0.32 at d = 1.9 delta, where it saturates toward c / g_ref), and its one constant came from
    // a reference geometry. And what the 3D loop measured instead of either, the relative field
    // error (Phi - c)/c, was not a length at all -- see relative_residual() in the header.
    //
    // THE SEARCH. Along u, the direction the level set lies in -- toward the complex from outside
    // (Phi < c), away from it from inside -- with s >= 0 the distance along u and
    // h(s) = sigma (Phi(p + s u) - c), sigma = sign(Phi(p) - c), so that h(0) > 0 and the root
    // sought is h's first zero; h'(s) = -grad Phi . n in both cases.
    //
    // FIRST PROPOSAL: the kernel's own answer, s_k = |b^-1(Phi(p)) - delta|, the root if one pair
    // is active along the ray, checked against the field itself at p + s_k u and accepted when
    // the residual there is within the tolerance below of the residual at p -- a secant step from
    // there would move less than 1e-4 of s_k. This is not a model the answer rests on: b is the
    // barrier object the pairs are evaluated with, the field decides, and where it disagrees (a
    // blend of several pairs) the search simply continues from the evaluated point. It is what
    // keeps the cost near two evaluations instead of seven. Measured on the cube: the Newton
    // search alone cost 6.7 us a call against 1.1 us for the value the loop read before, and the
    // smooth run (target_distance_rel 1e-2, front_conv_rel 1e-4, 10 threads, frames) took 886 s,
    // the added time in the swap and collapse passes, whose ops guards evaluate ring measures,
    // and in the frame writer; with this proposal 2.1 us a call and 98 s. On blends (two cubes
    // 1.5 delta apart, the notched cube) the proposal is off by up to 30% at 23% and 14% of the
    // points sampled near the level set, is refused there, and the search then agrees with a
    // brute-force first crossing along the same ray to 8.2e-5 of the length.
    //
    // THEN a safeguarded Newton on a bracket [lo, hi] with h(lo) > 0 >= h(hi): Newton's step
    // while it stays inside what is known, a bisection of the bracket otherwise.
    //
    // THE STEP CAP, while no hi is known: at most delta past lo. Where one pair is active every
    // point within delta of its feature has Phi >= b(delta) = c, and walking along n toward the
    // complex the distance falls at unit rate, so that set is a stretch of length delta in front
    // of the complex: a step of delta cannot jump over it, the first sample past the level set
    // lies short of the complex, and the search never reaches the mirror level set Phi = c on the
    // far side of the complex -- the failure that made a root search unusable in the placement
    // energy (8fb6e6be2d), where it was also warm-started from the previous root; this one never
    // is. Where several pairs blend (ESP's signed terms at a reentrant feature), n need not point
    // at the complex and the cap is a safeguard, not a proof. Moving away from the complex from
    // inside, Newton undershoots on the convex b and the cap never binds. No zero within dhat of
    // p: there is no level set to measure against there.
    //
    // THE TOLERANCE: relative, 1e-4 of the answer. Every consumer divides this length by a bar
    // and compares the ratio with 1 (or squares and averages it first), so a relative accuracy of
    // the length is the same relative accuracy of the ratio at every bar; an absolute tolerance
    // such as 1e-4 delta would exceed the bar itself once front_conv_rel is set below 1e-4 of
    // target_distance_rel, and the spec gives front_conv_rel no lower bound. A Newton stop leaves
    // an error far below it (quadratic convergence). A step that no longer moves the query point
    // ends the search as well: nothing finer is representable.
    constexpr double kRelTol = 1e-4;
    double phi0;
    VecD g0;
    value_gradient(p, phi0, g0);
    if (!std::isfinite(phi0) || !(phi0 > 0.))
        return false; // on the complex, or outside the support
    const double gn0 = g0.norm();
    if (!std::isfinite(gn0) || !(gn0 > 0.)) return false; // no direction to the level set
    const double f0 = phi0 - m_c;
    if (f0 == 0.) {
        t = 0.;
        return true;
    }
    const VecD n = g0 / gn0;
    const double sigma = f0 > 0. ? 1. : -1.;
    const VecD u = -sigma * n;
    const double h0 = std::abs(f0);
    double lo = 0., hi = std::numeric_limits<double>::infinity();
    double s = 0., h = h0, dh = -gn0;
    VecD q_s = p;

    {
        // b^-1(Phi(p)) on (0, dhat), where b falls monotonically from +infinity to 0: a
        // safeguarded scalar Newton on the barrier alone, no field evaluation.
        const ipc::Barrier& b = *m_impl->params.barrier;
        double dlo = 0., dhi = m_dhat, d = m_delta;
        for (int it = 0; it < 100; ++it) {
            const double r = b(d, m_dhat) - phi0;
            if (r == 0.) break;
            if (r > 0.) {
                dlo = d;
            } else {
                dhi = d;
            }
            double dn = d - r / b.first_derivative(d, m_dhat);
            if (!(dn > dlo && dn < dhi)) dn = 0.5 * (dlo + dhi);
            if (dn == d) break;
            d = dn;
        }
        const double sk = std::abs(d - m_delta);
        if (sk > 0. && sk <= m_delta) {
            const VecD q = p + sk * u;
            const double phi = value(q);
            if (std::isnan(phi)) return false;
            const double hk = sigma * (phi - m_c);
            if (hk == 0. || std::abs(hk) <= kRelTol * std::abs(h0 - hk)) {
                t = -sigma * sk;
                return true;
            }
            if (hk > 0.) {
                lo = sk;
            } else {
                hi = sk;
            }
            s = sk;
            h = hk;
            q_s = q;
            dh = -gradient(q).dot(n);
        }
    }

    for (int it = 0; it < 200; ++it) {
        const bool bracketed = std::isfinite(hi);
        if (!bracketed && lo >= m_dhat) return false;
        double sn = s - h / dh;
        bool newton = std::isfinite(sn) && dh < 0.;
        if (bracketed) {
            if (!(newton && sn > lo && sn < hi)) {
                sn = 0.5 * (lo + hi);
                newton = false;
            }
        } else {
            const double cap = lo + m_delta;
            if (!(newton && sn > lo && sn <= cap)) {
                sn = cap;
                newton = false;
            }
        }
        const VecD q = p + sn * u;
        if (q == q_s) break; // the step no longer moves the query point
        const double phi = value(q);
        if (std::isnan(phi)) return false;
        const double hn = sigma * (phi - m_c);
        if (hn > 0.) {
            lo = sn;
        } else {
            hi = sn;
        }
        const bool done = hn == 0. || (newton && std::abs(sn - s) <= kRelTol * sn) ||
                          (std::isfinite(hi) && hi - lo <= kRelTol * lo);
        s = sn;
        h = hn;
        q_s = q;
        if (done) break;
        dh = -gradient(q).dot(n);
    }
    t = -sigma * s;
    return true;
}

template <int DIM>
double SmoothOffsetPotential<DIM>::residual_length(const VecD& p) const
{
    double t;
    return level_set_distance(p, t) ? std::abs(t) : std::numeric_limits<double>::infinity();
}

template <int DIM>
double SmoothOffsetPotential<DIM>::relative_residual(const VecD& p) const
{
    double t;
    return level_set_distance(p, t) ? t / m_delta : std::numeric_limits<double>::quiet_NaN();
}


// ---------------------------------------------------------------------------------------------


template <int DIM>
OffsetEnergy<DIM>::OffsetEnergy(
    const std::shared_ptr<const OffsetPotential<DIM>>& potential,
    const double weight,
    const bool gauss_newton,
    const bool distance_residual,
    const bool one_sided)
    : m_potential(potential)
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
    , m_distance_residual(distance_residual)
    , m_one_sided(one_sided)
{}


template <int DIM>
void OffsetEnergy<DIM>::residual(const VecD& p, double& r, VecD& dr) const
{
    // one_sided: nothing beyond the level set (see the constructor).
    if (m_one_sided && !m_potential->is_inside_offset(p)) {
        r = 0.;
        dr.setZero();
        return;
    }
    const double c = std::max(m_potential->target_level(), 1e-300);
    if (m_distance_residual && !m_potential->is_euclidean()) {
        // The monotone length (Phi - c)/grad_ref in units of delta: exact at the level set,
        // single-valued everywhere, growing without bound toward the input. Not a root-found
        // distance: the smooth field has a second level set Phi = c inside the input, and a root
        // search can converge to it and measure the distance to the wrong side.
        const double delta = std::max(m_potential->delta(), 1e-300);
        const double g_ref = std::max(m_potential->level_set_slope(), 1e-300);
        r = (m_potential->value(p) - c) / (g_ref * delta);
        dr = m_potential->gradient(p) / (g_ref * delta);
        return;
    }
    // Normalised by the level: r = (Phi - c) / c, so the term is O(1) for every field and every
    // target_distance at the level set. For the Euclidean field this is exactly (d - delta)/delta,
    // and it stays the Euclidean residual under either flag.
    r = (m_potential->value(p) - c) / c;
    dr = m_potential->gradient(p) / c;
}

template <int DIM>
double OffsetEnergy<DIM>::value(const TVector& x)
{
    double r;
    VecD dr;
    residual(VecD(x.head(DIM)), r, dr);
    return m_weight * r * r;
}

template <int DIM>
void OffsetEnergy<DIM>::gradient(const TVector& x, TVector& gradv)
{
    double r;
    VecD dr;
    residual(VecD(x.head(DIM)), r, dr);
    gradv = 2. * m_weight * r * dr;
}

template <int DIM>
void OffsetEnergy<DIM>::hessian(const TVector& x, MatrixXd& hessian)
{
    const VecD p = x.head(DIM);
    double r;
    VecD dr;
    residual(p, r, dr);
    MatD H = 2. * m_weight * dr * dr.transpose();
    if (!m_gauss_newton && !(m_distance_residual && !m_potential->is_euclidean())) {
        const double c = std::max(m_potential->target_level(), 1e-300);
        H += 2. * m_weight * r * m_potential->hessian(p) / c;
    }
    hessian = H;
}


// ---------------------------------------------------------------------------------------------
// The Euclidean field.
// ---------------------------------------------------------------------------------------------

template <int DIM>
EuclideanOffsetPotential<DIM>::EuclideanOffsetPotential(
    const std::shared_ptr<SampleEnvelope>& envelope,
    const double delta)
    // No support limit: d is defined everywhere, so the runaway guard that exists for Phi's compact
    // support has nothing to catch. Infinity says so, rather than a large finite number something
    // might later compare against.
    : OffsetPotential<DIM>(delta, std::numeric_limits<double>::infinity())
    , m_envelope(envelope)
{
    if (!(delta > 0.)) {
        log_and_throw_error(
            "EuclideanOffsetPotential: target_distance must be positive, got {}",
            delta);
    }
    if (!m_envelope) {
        log_and_throw_error("EuclideanOffsetPotential: needs an envelope to query");
    }
    // No calibration: the level is the offset distance, where the smooth potential has to discover
    // its own level by evaluating Phi at distance delta from a flat reference.
    //
    // In units of target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta. The raw
    // distance made the placement's pull ~1/delta^2 weaker than the smooth potential's at the same
    // misplacement, weak enough for the small AMIPS term to hold a misplaced vertex at a balance
    // point. Every consumer works in ratios of value to c or divides by level_set_slope(), so
    // |value - c| / |grad| is still d - delta.
    m_c = 1.;
    m_grad_ref = 1. / delta;
}

template <int DIM>
EuclideanOffsetPotential<DIM>::EuclideanOffsetPotential(
    const std::shared_ptr<SimplicialComplexBVH>& bvh,
    const double delta)
    : OffsetPotential<DIM>(delta, std::numeric_limits<double>::infinity())
    , m_bvh(bvh)
{
    if constexpr (DIM != 2) {
        // The BVH's feature query is 2D; 3D still runs on its input-complex envelope. A runtime
        // check because the explicit instantiations below compile every member for both DIMs.
        log_and_throw_error("EuclideanOffsetPotential<3>: the BVH-backed path is 2D-only");
    }
    if (!(delta > 0.)) {
        log_and_throw_error(
            "EuclideanOffsetPotential: target_distance must be positive, got {}",
            delta);
    }
    if (!m_bvh) {
        log_and_throw_error("EuclideanOffsetPotential: needs a BVH to query");
    }
    // In units of target_distance: value = d / delta, level c = 1, so |grad| = 1 / delta. See the
    // envelope-backed constructor above for why the raw distance is not used.
    m_c = 1.;
    m_grad_ref = 1. / delta;
}

template <int DIM>
void EuclideanOffsetPotential<DIM>::nearest_feature(const VecD& p, VecD& foot, int& dim, VecD& dir)
    const
{
    if constexpr (DIM == 2) {
        bool on_corner = false;
        int feature_id = -1;
        Eigen::Vector2d seg_normal;
        // Same query, either engine: the two implementations of it are identical.
        if (m_bvh) {
            m_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        } else {
            m_envelope->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id);
        }
        // The 2D query reports the segment normal, while the Hessian below is cased on the
        // direction along the feature: a segment interior is dim 1 with the tangent, obtained by
        // rotating the normal a quarter turn.
        dim = on_corner ? 0 : 1;
        dir = on_corner ? VecD::Zero().eval() : VecD(-seg_normal.y(), seg_normal.x());
    } else {
        long long feature_id = -1;
        m_envelope->nearest_point_feature(p, foot, dim, dir, feature_id);
    }

    // A degenerate segment is a point, not an edge: both SimplicialComplexBVH and the envelope
    // carry an isolated input vertex as the pseudo-edge (i, i), so a query near one comes back as
    // an edge-interior hit whose direction is whatever normalising a zero vector produced, and the
    // edge Hessian would subtract a meaningless t t^T. Demote it to the vertex case.
    if (dim == 1 && !(dir.norm() > 0.5)) {
        dim = 0;
        dir = VecD::Zero();
    }
}

template <int DIM>
double EuclideanOffsetPotential<DIM>::value(const VecD& p) const
{
    if constexpr (DIM == 2) {
        if (m_bvh) {
            // The distance to the complex's curve, through the feature query -- never
            // squared_dist(), which measures the solid complex and is identically zero inside a
            // solid region. The two agree everywhere the offset lives, but value() must not
            // silently change meaning inside.
            Eigen::Vector2d foot;
            bool on_corner = false;
            Eigen::Vector2d seg_normal;
            int feature_id = -1;
            return std::sqrt(
                       m_bvh->nearest_point_feature(p, foot, on_corner, seg_normal, feature_id)) /
                   m_delta;
        }
    }
    return std::sqrt(m_envelope->squared_distance(p)) / m_delta;
}

template <int DIM>
typename EuclideanOffsetPotential<DIM>::VecD EuclideanOffsetPotential<DIM>::gradient(
    const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);

    const VecD r = p - foot;
    const double d = r.norm();
    // On the complex the gradient of d does not exist, since every direction increases it equally.
    // Zero contributes no offset force, so such a vertex is moved by the quality term alone. The
    // front smoother excludes input-complex vertices from the offset term and the criterion books
    // them as pinned, so this case is a backstop.
    if (!(d > 1e-14)) {
        return VecD::Zero();
    }
    return r / (d * m_delta);
}

template <int DIM>
typename EuclideanOffsetPotential<DIM>::MatD EuclideanOffsetPotential<DIM>::hessian(
    const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);

    const VecD r = p - foot;
    const double d = r.norm();
    if (!(d > 1e-14)) {
        return MatD::Zero();
    }
    const VecD u = r / d;

    // grad^2 d, by feature kind. Transcribed from ExactDistanceEnergy2D/3D, which state the
    // Hessian of d^2; grad^2(d^2) = 2 (grad d grad d^T + d grad^2 d) converts one to the other.
    // See the class comment for the table.
    if (dim == DIM - 1) {
        // Face interior in 3D, segment interior in 2D: d is linear in p there, so no curvature.
        return MatD::Zero();
    }
    if (dim == 1) {
        // 3D edge interior: free along the edge, curved around it.
        return (MatD::Identity() - dir * dir.transpose() - u * u.transpose()) / (d * m_delta);
    }
    // Vertex: distance to a point, curved in every direction but radially.
    return (MatD::Identity() - u * u.transpose()) / (d * m_delta);
}

template <int DIM>
std::string EuclideanOffsetPotential<DIM>::describe_active(const VecD& p) const
{
    VecD foot = VecD::Zero(), dir = VecD::Zero();
    int dim = -1;
    nearest_feature(p, foot, dim, dir);
    static constexpr std::array<const char*, 3> kinds = {{"vertex", "edge interior", "face"}};
    return fmt::format(
        "nearest feature: {} at ({}), d = {:.6g}, level = {:.6g}, residual = {:.6g}",
        kinds[size_t(std::clamp(dim, 0, 2))],
        fmt::join(std::vector<double>(foot.data(), foot.data() + DIM), ", "),
        (p - foot).norm(),
        m_c,
        residual_length(p));
}


template <int DIM>
OffsetPotential<DIM>::~OffsetPotential() = default;


template class OffsetPotential<2>;
template class OffsetPotential<3>;
template class SmoothOffsetPotential<2>;
template class SmoothOffsetPotential<3>;
template class EuclideanOffsetPotential<2>;
template class EuclideanOffsetPotential<3>;
template class OffsetEnergy<2>;
template class OffsetEnergy<3>;

RestAMIPSEnergy2D::RestAMIPSEnergy2D(std::vector<Cell> cells, const double weight)
    : m_cells(std::move(cells))
    , m_weight(weight)
{}

bool RestAMIPSEnergy2D::cell_F(
    const Eigen::Vector2d& x,
    const Cell& c,
    Eigen::Matrix2d& F,
    double& d) const
{
    Eigen::Matrix2d A;
    A.col(0) = c.q1 - x;
    A.col(1) = c.q2 - x;
    F = A * c.rest_inv;
    d = F.determinant();
    return d > 0.;
}

double RestAMIPSEnergy2D::value(const TVector& x)
{
    double E = 0.;
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) return std::nan("");
        E += m_weight * F.squaredNorm() / d;
    }
    return E;
}

void RestAMIPSEnergy2D::gradient(const TVector& x, TVector& gradv)
{
    // dE/dF = 2F/d - (e/d) F^-T, in closed form for 2x2; dF/dx_k = -e_k * r^T with
    // r = Rinv^T (1,1)^T, so grad_x = -(dE/dF) r summed over cells.
    Eigen::Vector2d G = Eigen::Vector2d::Zero();
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) {
            gradv = Eigen::Vector2d::Zero(); // invalid point: the line search never accepts it
            return;
        }
        const double e = F.squaredNorm();
        Eigen::Matrix2d FinvT;
        FinvT << F(1, 1), -F(1, 0), -F(0, 1), F(0, 0);
        FinvT /= d;
        const Eigen::Matrix2d dEdF = 2. / d * F - (e / d) * FinvT;
        const Eigen::Vector2d r = c.rest_inv.transpose() * Eigen::Vector2d::Ones();
        G += -m_weight * (dEdF * r);
    }
    gradv = G;
}

void RestAMIPSEnergy2D::hessian(const TVector& x, MatrixXd& hessian)
{
    // F is affine in x, so H_x = M^T H_F M exactly, with M the constant 4x2 dvecF/dx
    // (column-major vec) and H_F the closed-form Hessian of e/d in F:
    //   dE = e'/d - e d'/d^2,  d2E = e''/d - (e' d'^T + d' e'^T)/d^2 - e d''/d^2
    //        + 2 e (d' d'^T)/d^3,
    // e' = 2 vecF, e'' = 2I, d' = (F11, -F01, -F10, F00), d'' = the constant K.
    Eigen::Matrix2d Hx = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d F;
    double d;
    Eigen::Matrix4d K = Eigen::Matrix4d::Zero();
    K(0, 3) = K(3, 0) = 1.;
    K(1, 2) = K(2, 1) = -1.;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(2), c, F, d)) continue; // invalid point: contribute nothing
        const double e = F.squaredNorm();
        Eigen::Vector4d vF(F(0, 0), F(1, 0), F(0, 1), F(1, 1));
        Eigen::Vector4d dd(F(1, 1), -F(0, 1), -F(1, 0), F(0, 0));
        const Eigen::Vector4d de = 2. * vF;
        Eigen::Matrix4d HF = (2. / d) * Eigen::Matrix4d::Identity();
        HF -= (de * dd.transpose() + dd * de.transpose()) / (d * d);
        HF += (2. * e / (d * d * d)) * (dd * dd.transpose());
        HF -= (e / (d * d)) * K;
        const Eigen::Vector2d r = c.rest_inv.transpose() * Eigen::Vector2d::Ones();
        Eigen::Matrix<double, 4, 2> M = Eigen::Matrix<double, 4, 2>::Zero();
        // dF(i,j)/dx_k = -Rinv row-sum of column j when k == i: vec index 2j + i.
        for (int j = 0; j < 2; ++j) {
            for (int i = 0; i < 2; ++i) {
                M(2 * j + i, i) = -r(j);
            }
        }
        Hx += m_weight * (M.transpose() * HF * M);
    }
    hessian = Hx;
}

bool RestAMIPSEnergy2D::is_step_valid(const TVector& /*x0*/, const TVector& x1)
{
    Eigen::Matrix2d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x1.head(2), c, F, d)) return false;
    }
    return true;
}


// ---------------------------------------------------------------------------------------------
// The 3D twin of the 2D rest-shape energy above.
// ---------------------------------------------------------------------------------------------

RestAMIPSEnergy3D::RestAMIPSEnergy3D(std::vector<Cell> cells, const double weight)
    : m_cells(std::move(cells))
    , m_weight(weight)
{}

bool RestAMIPSEnergy3D::cell_F(
    const Eigen::Vector3d& x,
    const Cell& c,
    Eigen::Matrix3d& F,
    double& d) const
{
    Eigen::Matrix3d A;
    A.col(0) = c.q1 - x;
    A.col(1) = c.q2 - x;
    A.col(2) = c.q3 - x;
    F = A * c.rest_inv;
    d = F.determinant();
    return d > 0.;
}

double RestAMIPSEnergy3D::value(const TVector& x)
{
    double E = 0.;
    Eigen::Matrix3d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(3), c, F, d)) return std::nan("");
        E += m_weight * F.squaredNorm() / std::cbrt(d * d);
    }
    return E;
}

namespace {
/// The cofactor matrix, dE/dF's second term: d det(F) / dF = det(F) F^-T.
inline Eigen::Matrix3d cofactor3(const Eigen::Matrix3d& F)
{
    Eigen::Matrix3d C;
    C.col(0) = F.col(1).cross(F.col(2));
    C.col(1) = F.col(2).cross(F.col(0));
    C.col(2) = F.col(0).cross(F.col(1));
    return C;
}
inline int levi_civita(const int i, const int j, const int k)
{
    if (i == j || j == k || i == k) return 0;
    return ((j - i + 3) % 3 == 1) ? 1 : -1;
}
} // namespace

void RestAMIPSEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    // E = e / d^(2/3): dE/dF = 2 F d^(-2/3) - (2/3) e d^(-5/3) cof(F). F = Q Rinv - x r^T with
    // r = Rinv^T (1,1,1)^T, so dF/dx_k = -e_k r^T and grad_x = -(dE/dF) r, summed over cells.
    Eigen::Vector3d G = Eigen::Vector3d::Zero();
    Eigen::Matrix3d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(3), c, F, d)) {
            gradv = Eigen::Vector3d::Zero(); // invalid point: the line search never accepts it
            return;
        }
        const double e = F.squaredNorm();
        const double d23 = std::cbrt(d * d);
        const Eigen::Matrix3d dEdF = (2. / d23) * F - (2. / 3.) * (e / (d23 * d)) * cofactor3(F);
        const Eigen::Vector3d r = c.rest_inv.transpose() * Eigen::Vector3d::Ones();
        G += -m_weight * (dEdF * r);
    }
    gradv = G;
}

void RestAMIPSEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    // F is affine in x, so H_x = M^T H_F M exactly, with M the constant 9x3 dvecF/dx and H_F
    // the closed-form Hessian of e / d^(2/3) in F (column-major vec):
    //   dE  = e' d^(-2/3) - (2/3) e d^(-5/3) d'
    //   d2E = e'' d^(-2/3) - (2/3) d^(-5/3) (e' d'^T + d' e'^T) + (10/9) e d^(-8/3) d' d'^T
    //         - (2/3) e d^(-5/3) d''
    // e' = 2 vecF, e'' = 2 I, d' = vec(cof F), d''_{(ij),(kl)} = eps_ikm eps_jln F_mn.
    Eigen::Matrix3d Hx = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x.head(3), c, F, d)) continue; // invalid point: contribute nothing
        const double e = F.squaredNorm();
        const double d23 = std::cbrt(d * d);
        const double d53 = d23 * d, d83 = d23 * d * d;
        Eigen::Matrix<double, 9, 1> vF, dd;
        const Eigen::Matrix3d C = cofactor3(F);
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                vF(3 * j + i) = F(i, j);
                dd(3 * j + i) = C(i, j);
            }
        }
        const Eigen::Matrix<double, 9, 1> de = 2. * vF;
        Eigen::Matrix<double, 9, 9> K = Eigen::Matrix<double, 9, 9>::Zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    for (int l = 0; l < 3; ++l) {
                        double v = 0.;
                        for (int m = 0; m < 3; ++m)
                            for (int n = 0; n < 3; ++n)
                                v += levi_civita(i, k, m) * levi_civita(j, l, n) * F(m, n);
                        K(3 * j + i, 3 * l + k) = v;
                    }
        Eigen::Matrix<double, 9, 9> HF = (2. / d23) * Eigen::Matrix<double, 9, 9>::Identity();
        HF -= (2. / 3.) / d53 * (de * dd.transpose() + dd * de.transpose());
        HF += (10. / 9.) * e / d83 * (dd * dd.transpose());
        HF -= (2. / 3.) * e / d53 * K;
        const Eigen::Vector3d r = c.rest_inv.transpose() * Eigen::Vector3d::Ones();
        Eigen::Matrix<double, 9, 3> M = Eigen::Matrix<double, 9, 3>::Zero();
        // dF(i,j)/dx_k = -r_j when k == i: vec index 3j + i.
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                M(3 * j + i, i) = -r(j);
            }
        }
        Hx += m_weight * (M.transpose() * HF * M);
    }
    hessian = Hx;
}

bool RestAMIPSEnergy3D::is_step_valid(const TVector& /*x0*/, const TVector& x1)
{
    Eigen::Matrix3d F;
    double d;
    for (const Cell& c : m_cells) {
        if (!cell_F(x1.head(3), c, F, d)) return false;
    }
    return true;
}


// ---------------------------------------------------------------------------------------------
// CubedAMIPSEnergy3D
// ---------------------------------------------------------------------------------------------

CubedAMIPSEnergy3D::CubedAMIPSEnergy3D(
    std::vector<std::array<double, 12>> cells,
    const double weight)
    : m_cells(std::move(cells))
    , m_weight(weight)
{}

namespace {
/// A cell of CubedAMIPSEnergy3D with the moving vertex placed at x.
std::array<double, 12> cubed_amips_cell_at(std::array<double, 12> c, const Eigen::VectorXd& x)
{
    c[0] = x[0];
    c[1] = x[1];
    c[2] = x[2];
    return c;
}
} // namespace

namespace {
/// vol AMIPS^3 of a CubedAMIPSEnergy3D cell (moving vertex x first) with AMIPS a (first power) and
/// its derivatives ga, ha in x: f = (sqrt2/12) T^(3/2) a^(3/2), T = (1/2) sum over the six edges of
/// |e|^2, grad T = sum over the three edges at x of (x - p_j), hess T = 3 I. The volume never comes
/// from a determinant: AMIPS = T / det(J)^(2/3) gives vol = (sqrt2/12) (T/a)^(3/2), and a
/// floating-point determinant of a nearly flat cell can come out 0 while its AMIPS is huge (see
/// TopoOffsetTetMesh::cell_amips_term()). need: 0 value, 1 + gradient, 2 + Hessian.
double cubed_amips_vol_term(
    const std::array<double, 12>& c,
    const double a,
    const Eigen::Vector3d& ga,
    const Eigen::Matrix3d& ha,
    const int need,
    Eigen::Vector3d& g,
    Eigen::Matrix3d& h)
{
    const Eigen::Vector3d p[4] = {
        Eigen::Vector3d(c[0], c[1], c[2]),
        Eigen::Vector3d(c[3], c[4], c[5]),
        Eigen::Vector3d(c[6], c[7], c[8]),
        Eigen::Vector3d(c[9], c[10], c[11])};
    double T = 0.;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) T += (p[i] - p[j]).squaredNorm();
    T *= 0.5;
    const double k = std::sqrt(2.) / 12.;
    const double sT = std::sqrt(T), sa = std::sqrt(a);
    const double f = k * T * sT * a * sa;
    if (need >= 1) {
        const Eigen::Vector3d gT = 3. * p[0] - p[1] - p[2] - p[3];
        g = k * 1.5 * (sT * a * sa * gT + T * sT * sa * ga);
        if (need >= 2) {
            h = k * 1.5 *
                (0.5 / sT * a * sa * gT * gT.transpose() +
                 1.5 * sT * sa * (gT * ga.transpose() + ga * gT.transpose()) +
                 3. * sT * a * sa * Eigen::Matrix3d::Identity() +
                 0.5 * T * sT / sa * ga * ga.transpose() + T * sT * sa * ha);
        }
    }
    return f;
}
} // namespace

double CubedAMIPSEnergy3D::value(const TVector& x)
{
    double res = 0.;
    for (const auto& c0 : m_cells) {
        const auto c = cubed_amips_cell_at(c0, x);
        const double a = wmtk::AMIPS_energy(c);
        if (m_volume_weighted) {
            Eigen::Vector3d g;
            Eigen::Matrix3d h;
            res += cubed_amips_vol_term(
                c,
                a,
                Eigen::Vector3d::Zero(),
                Eigen::Matrix3d::Zero(),
                0,
                g,
                h);
        } else {
            res += a * a * a;
        }
    }
    return m_weight * res;
}

void CubedAMIPSEnergy3D::gradient(const TVector& x, TVector& gradv)
{
    gradv.setZero(3);
    Eigen::Vector3d g;
    for (const auto& c0 : m_cells) {
        const auto c = cubed_amips_cell_at(c0, x);
        const double a = wmtk::AMIPS_energy(c);
        wmtk::AMIPS_jacobian(c, g);
        if (m_volume_weighted) {
            Eigen::Vector3d gf;
            Eigen::Matrix3d hf;
            cubed_amips_vol_term(c, a, g, Eigen::Matrix3d::Zero(), 1, gf, hf);
            gradv += gf;
        } else {
            gradv += 3. * a * a * g;
        }
    }
    gradv *= m_weight;
}

void CubedAMIPSEnergy3D::hessian(const TVector& x, MatrixXd& hessian)
{
    hessian.setZero(3, 3);
    Eigen::Vector3d g;
    Eigen::Matrix3d h;
    for (const auto& c0 : m_cells) {
        const auto c = cubed_amips_cell_at(c0, x);
        const double a = wmtk::AMIPS_energy(c);
        wmtk::AMIPS_jacobian(c, g);
        wmtk::AMIPS_hessian(c, h);
        if (m_volume_weighted) {
            Eigen::Vector3d gf;
            Eigen::Matrix3d hf;
            cubed_amips_vol_term(c, a, g, h, 2, gf, hf);
            hessian += hf;
        } else {
            hessian += 3. * a * a * h + 6. * a * g * g.transpose();
        }
    }
    hessian *= m_weight;
}

bool CubedAMIPSEnergy3D::is_step_valid(const TVector& /*x0*/, const TVector& x1)
{
    // The engine's AMIPSEnergy3D::is_step_valid: the moved vertex may not invert a cell.
    const Eigen::Vector3d p0 = x1.head(3);
    for (const auto& c : m_cells) {
        if (!wmtk::utils::orient3d(
                p0,
                Eigen::Vector3d(c[3], c[4], c[5]),
                Eigen::Vector3d(c[6], c[7], c[8]),
                Eigen::Vector3d(c[9], c[10], c[11]))) {
            return false;
        }
    }
    return true;
}


// ---------------------------------------------------------------------------------------------
// InputTriangles
// ---------------------------------------------------------------------------------------------

InputTriangles::InputTriangles(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    : m_V(V)
    , m_F(F)
{
    m_bvh.init(m_V, m_F, 1e-6);
    // min_mean_distance()'s tree, median split on the longest axis of the triangles' centroids.
    const int nf = int(m_F.rows());
    if (nf == 0) return;
    std::vector<Eigen::Vector3d> centroid(static_cast<size_t>(nf));
    for (int f = 0; f < nf; ++f) {
        centroid[size_t(f)] =
            (m_V.row(m_F(f, 0)) + m_V.row(m_F(f, 1)) + m_V.row(m_F(f, 2))).head<3>().transpose() /
            3.;
    }
    const Eigen::Vector3d lo = m_V.leftCols<3>().colwise().minCoeff().transpose();
    const Eigen::Vector3d hi = m_V.leftCols<3>().colwise().maxCoeff().transpose();
    m_order.resize(size_t(nf));
    for (int f = 0; f < nf; ++f) m_order[size_t(f)] = f;
    m_nodes.reserve(size_t(2 * nf));
    m_pad = 1e-9 * std::max((hi - lo).norm(), 1e-300);
    build_node(0, nf, centroid, m_pad);
}

int InputTriangles::build_node(
    const int begin,
    const int end,
    const std::vector<Eigen::Vector3d>& centroid,
    const double pad)
{
    const int id = int(m_nodes.size());
    m_nodes.emplace_back();
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo, clo = lo, chi = -lo;
    for (int j = begin; j < end; ++j) {
        const int f = m_order[size_t(j)];
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d v = m_V.row(m_F(f, c)).head<3>().transpose();
            lo = lo.cwiseMin(v);
            hi = hi.cwiseMax(v);
        }
        clo = clo.cwiseMin(centroid[size_t(f)]);
        chi = chi.cwiseMax(centroid[size_t(f)]);
    }
    m_nodes[size_t(id)].lo = lo - Eigen::Vector3d::Constant(pad);
    m_nodes[size_t(id)].hi = hi + Eigen::Vector3d::Constant(pad);
    m_nodes[size_t(id)].begin = begin;
    m_nodes[size_t(id)].end = end;
    if (end - begin <= 4) return id;
    int axis = 0;
    (chi - clo).maxCoeff(&axis);
    const int mid = (begin + end) / 2;
    std::nth_element(
        m_order.begin() + begin,
        m_order.begin() + mid,
        m_order.begin() + end,
        [&](const int a, const int b) {
            return centroid[size_t(a)][axis] < centroid[size_t(b)][axis] ||
                   (centroid[size_t(a)][axis] == centroid[size_t(b)][axis] && a < b);
        });
    const int left = build_node(begin, mid, centroid, pad);
    const int right = build_node(mid, end, centroid, pad);
    m_nodes[size_t(id)].left = left;
    m_nodes[size_t(id)].right = right;
    return id;
}

double InputTriangles::min_mean_distance(
    const Eigen::Vector3d* p,
    const int n,
    int64_t& best,
    const int64_t hint) const
{
    double m = std::numeric_limits<double>::infinity();
    best = -1;
    const auto offer = [&](const int64_t t) {
        double s = 0.;
        for (int i = 0; i < n; ++i) s += distance(t, p[i]);
        s /= n;
        if (s < m || (s == m && t < best)) m = s, best = t;
    };
    // A hint seeds the bound; without one the walk starts unbounded. The result does not depend on
    // the seed (see the walk below). MEASURED 2026-10-09, serial cube, 3 turns: seeding with each
    // point's nearest triangle (n tree queries) took 24.3 s against 20.6 s unseeded, same output.
    if (hint >= 0) offer(hint);
    if (m_nodes.empty()) return m;
    const auto lower = [&](const Node& nd) {
        double s = 0.;
        for (int i = 0; i < n; ++i) {
            s += (nd.lo - p[i]).cwiseMax(p[i] - nd.hi).cwiseMax(0.).norm();
        }
        return s / n;
    };
    // Depth first, the nearer child first; a node whose bound EXCEEDS the best is skipped (equal is
    // visited, so that the lowest index wins a tie whatever the starting point).
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > m) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) offer(m_order[size_t(j)]);
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    return m;
}

void InputTriangles::ball_candidates(
    const Eigen::Vector3d& c,
    const double r,
    const std::array<Eigen::Vector3d, 3>& q,
    std::vector<BallCandidate>& out) const
{
    // f_P(x) = (1/4)[d_P(x) + d_P(q1) + d_P(q2) + d_P(q3)], the mean min_mean_distance() takes of
    // {x, q1, q2, q3}, summed in that order. For |x - c| <= R, d_P being 1-Lipschitz,
    //     LB_P = (1/4)[max(0, d_P(c) - R) + d_P(q1) + d_P(q2) + d_P(q3)] <= f_P(x),
    //     f_Q(x) <= (1/4)[d_Q(c) + R + d_Q(q1) + d_Q(q2) + d_Q(q3)] = UB_Q   for every Q.
    // So with U = min over Q of UB_Q, a triangle P with LB_P > U has f_P(x) > U >= min_Q f_Q(x):
    // it is neither the minimiser nor tied with it. Every other triangle is kept, so the min over
    // the kept ones, scanned in increasing index with min_mean_distance()'s comparison, is its
    // min and its minimiser, the lowest index on a tie -- at every x of the ball.
    // In floating point: each bound is summed in f_P's order, its terms no larger (LB) or no
    // smaller (UB) than f_P's own, and rounding is monotone, so LB_P <= f_P(x) <= UB_P hold for
    // the computed values as long as the computed d_P moves by at most R between c and x. Hence
    // R = r (1 + 1e-9) + 2 pad: the computed (x - c).norm() <= r leaves the true |x - c| within a
    // few ulps of r, far below 1e-9 r, and the computed d_P is within pad of the exact one at
    // either point -- the allowance the boxes already rest on (a box distance never exceeds a
    // triangle distance). A triangle whose distance at c is NaN (degenerate) is kept: nothing is
    // certified about it.
    // The search is min_mean_distance()'s: a node's bound, LB with its box in place of P, is at
    // most f_P(x) for every P inside and every x of the ball (a box distance is 1-Lipschitz too,
    // and at most d_P), so a node whose bound exceeds the smallest UB_Q so far holds no minimiser
    // and is skipped.
    out.clear();
    if (m_nodes.empty()) return;
    const double R = r * (1. + 1e-9) + 2. * m_pad;
    const auto box = [](const Node& nd, const Eigen::Vector3d& p) {
        return (nd.lo - p).cwiseMax(p - nd.hi).cwiseMax(0.).norm();
    };
    const auto lower = [&](const Node& nd) {
        double s = 0.;
        s += std::max(0., box(nd, c) - R);
        for (const Eigen::Vector3d& p : q) s += box(nd, p);
        return s / 4;
    };
    double U = std::numeric_limits<double>::infinity();
    std::vector<std::pair<double, BallCandidate>> kept; // LB_P, P
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > U) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) {
                BallCandidate cd;
                cd.tri = m_order[size_t(j)];
                const double d0 = distance(cd.tri, c);
                cd.d1 = distance(cd.tri, q[0]);
                cd.d2 = distance(cd.tri, q[1]);
                cd.d3 = distance(cd.tri, q[2]);
                double lo = 0., hi = 0.;
                lo += std::isnan(d0) ? d0 : std::max(0., d0 - R);
                lo += cd.d1;
                lo += cd.d2;
                lo += cd.d3;
                lo /= 4;
                hi += d0 + R;
                hi += cd.d1;
                hi += cd.d2;
                hi += cd.d3;
                hi /= 4;
                if (hi < U) U = hi;
                if (!(lo > U)) kept.push_back({lo, cd});
            }
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    for (const auto& [lo, cd] : kept) {
        if (!(lo > U)) out.push_back(cd);
    }
    std::sort(out.begin(), out.end(), [](const BallCandidate& a, const BallCandidate& b) {
        return a.tri < b.tri;
    });
}

int64_t InputTriangles::nearest(const Eigen::Vector3d& p) const
{
    // SimpleBVH's nearest_facet returns its own (reordered) facet index; its box query returns
    // F's. So: the nearest distance from the BVH, then every triangle whose box reaches within it,
    // and the closest of those by distance() -- the lowest index on a tie.
    Eigen::Vector3d q;
    double sq = 0.;
    m_bvh.nearest_facet(p, q, sq);
    const double r = std::sqrt(sq) * (1. + 1e-12) + 1e-12;
    std::vector<unsigned int> list;
    m_bvh.intersect_box(p - Eigen::Vector3d::Constant(r), p + Eigen::Vector3d::Constant(r), list);
    int64_t best = -1;
    double bd = std::numeric_limits<double>::infinity();
    for (const unsigned int t : list) {
        const double d = distance(int64_t(t), p);
        if (d < bd || (d == bd && int64_t(t) < best)) bd = d, best = int64_t(t);
    }
    return best;
}

namespace {

/// The point of triangle abc closest to p and the feature it lies on (Ericson, Real-Time
/// Collision Detection 5.1.5): kind 0 a vertex, 1 an edge of direction `edge`, 2 the interior.
Eigen::Vector3d triangle_foot(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& p,
    int& kind,
    Eigen::Vector3d& edge)
{
    const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
    Eigen::Vector3d foot;
    kind = 2;
    edge = Eigen::Vector3d::Zero();
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    const Eigen::Vector3d cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    const double vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
    if (d1 <= 0. && d2 <= 0.) {
        foot = a, kind = 0;
    } else if (d3 >= 0. && d4 <= d3) {
        foot = b, kind = 0;
    } else if (vc <= 0. && d1 >= 0. && d3 <= 0.) {
        foot = a + d1 / (d1 - d3) * ab, kind = 1, edge = ab;
    } else if (d6 >= 0. && d5 <= d6) {
        foot = c, kind = 0;
    } else if (vb <= 0. && d2 >= 0. && d6 <= 0.) {
        foot = a + d2 / (d2 - d6) * ac, kind = 1, edge = ac;
    } else if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.) {
        foot = b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b), kind = 1, edge = c - b;
    } else {
        const double den = 1. / (va + vb + vc);
        foot = a + ab * (vb * den) + ac * (vc * den);
    }
    return foot;
}

/// The distance between segments p1 q1 and p2 q2: the closest pair of their lines clamped to
/// both segments (Ericson, Real-Time Collision Detection 5.1.9). A zero-length segment is its
/// point; parallel segments (a e - b^2 = 0, or below 0 by rounding) start from s = 0, and the
/// clamping then finds the pair at an endpoint, where parallel segments attain their distance.
double segment_distance(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
    const Eigen::Vector3d d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const double a = d1.squaredNorm(), e = d2.squaredNorm(), f = d2.dot(r);
    if (a == 0. && e == 0.) return r.norm();
    double s = 0., t = 0.;
    if (a == 0.) {
        t = std::clamp(f / e, 0., 1.);
    } else {
        const double c = d1.dot(r);
        if (e == 0.) {
            s = std::clamp(-c / a, 0., 1.);
        } else {
            const double b = d1.dot(d2), denom = a * e - b * b;
            if (denom > 0.) s = std::clamp((b * f - c * e) / denom, 0., 1.);
            t = (b * s + f) / e;
            if (t < 0.) {
                t = 0.;
                s = std::clamp(-c / a, 0., 1.);
            } else if (t > 1.) {
                t = 1.;
                s = std::clamp((b - c) / a, 0., 1.);
            }
        }
    }
    return ((p1 + s * d1) - (p2 + t * d2)).norm();
}

/// Whether segment pq crosses triangle abc: p and q strictly on opposite sides of its plane, and
/// the line pq through the closed triangle (the three orientations of pq against its edges of
/// one sign or 0). Exact signs: a crossing is never decided on rounding. A contact with p or q on
/// the plane is not a crossing here; it is a corner-to-triangle distance at 0.
bool segment_crosses_triangle(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    using wmtk::utils::predicates::orient3d;
    const int sp = int(orient3d(a, b, c, p)), sq = int(orient3d(a, b, c, q));
    if (sp * sq >= 0) return false;
    const int s1 = int(orient3d(p, q, a, b)), s2 = int(orient3d(p, q, b, c)),
              s3 = int(orient3d(p, q, c, a));
    return (s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0);
}

} // namespace

double InputTriangles::distance(
    const int64_t tri,
    const Eigen::Vector3d& p,
    Eigen::Vector3d* grad,
    Eigen::Matrix3d* hess) const
{
    const Eigen::Vector3d a = m_V.row(m_F(tri, 0)).head<3>().transpose();
    const Eigen::Vector3d b = m_V.row(m_F(tri, 1)).head<3>().transpose();
    const Eigen::Vector3d c = m_V.row(m_F(tri, 2)).head<3>().transpose();
    int kind = 2; // 0 vertex, 1 edge (direction `edge`), 2 interior
    Eigen::Vector3d edge;
    const Eigen::Vector3d foot = triangle_foot(a, b, c, p, kind, edge);
    const double d = (p - foot).norm();
    if (grad) *grad = d > 0. ? Eigen::Vector3d((p - foot) / d) : Eigen::Vector3d::Zero();
    if (hess) {
        hess->setZero();
        if (d > 0. && kind < 2) {
            const Eigen::Vector3d u = (p - foot) / d;
            *hess = Eigen::Matrix3d::Identity() - u * u.transpose();
            if (kind == 1) {
                const Eigen::Vector3d e = edge.normalized();
                *hess -= e * e.transpose();
            }
            *hess /= d;
        }
    }
    return d;
}

double InputTriangles::triangle_distance(
    const int64_t P,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c) const
{
    const std::array<Eigen::Vector3d, 3> A = {
        {m_V.row(m_F(P, 0)).head<3>().transpose(),
         m_V.row(m_F(P, 1)).head<3>().transpose(),
         m_V.row(m_F(P, 2)).head<3>().transpose()}};
    const std::array<Eigen::Vector3d, 3> B = {{a, b, c}};
    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3;
        if (segment_crosses_triangle(A[size_t(i)], A[size_t(j)], a, b, c) ||
            segment_crosses_triangle(B[size_t(i)], B[size_t(j)], A[0], A[1], A[2])) {
            return 0.;
        }
    }
    double m = std::numeric_limits<double>::infinity();
    int kind = 2;
    Eigen::Vector3d edge;
    for (int i = 0; i < 3; ++i) {
        m = std::min(m, distance(P, B[size_t(i)]));
        m = std::min(m, (A[size_t(i)] - triangle_foot(a, b, c, A[size_t(i)], kind, edge)).norm());
        for (int k = 0; k < 3; ++k) {
            m = std::min(
                m,
                segment_distance(
                    A[size_t(i)],
                    A[size_t((i + 1) % 3)],
                    B[size_t(k)],
                    B[size_t((k + 1) % 3)]));
        }
    }
    return m;
}

double InputTriangles::min_triangle_distance(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c) const
{
    double m = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d* q : {&a, &b, &c}) m = std::min(m, distance(nearest(*q), *q));
    if (m_nodes.empty()) return m;
    // The distance between abc's box and a node's: a lower bound on triangle_distance() for every
    // triangle inside the node, abc lying in its own box.
    const Eigen::Vector3d lo = a.cwiseMin(b).cwiseMin(c), hi = a.cwiseMax(b).cwiseMax(c);
    const auto lower = [&](const Node& nd) {
        return (nd.lo - hi).cwiseMax(lo - nd.hi).cwiseMax(0.).norm();
    };
    // Depth first, the nearer child first; a node whose bound exceeds the best is skipped.
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > m) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) {
                m = std::min(m, triangle_distance(m_order[size_t(j)], a, b, c));
            }
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    return m;
}

double InputTriangles::min_max_corner_distance(const Eigen::Vector3d* p, const int n) const
{
    double m = std::numeric_limits<double>::infinity();
    const auto offer = [&](const int64_t t) {
        double s = 0.;
        for (int i = 0; i < n; ++i) s = std::max(s, distance(t, p[i]));
        m = std::min(m, s);
    };
    for (int i = 0; i < n; ++i) offer(nearest(p[i]));
    if (m_nodes.empty()) return m;
    const auto lower = [&](const Node& nd) {
        double s = 0.;
        for (int i = 0; i < n; ++i) {
            s = std::max(s, (nd.lo - p[i]).cwiseMax(p[i] - nd.hi).cwiseMax(0.).norm());
        }
        return s;
    };
    // Depth first, the nearer child first; a node whose bound exceeds the best is skipped.
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > m) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) offer(m_order[size_t(j)]);
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    return m;
}

// ---------------------------------------------------------------------------------------------
// BandVolumeEnergy3D
// ---------------------------------------------------------------------------------------------

BandVolumeEnergy3D::BandVolumeEnergy3D(
    const std::shared_ptr<const OffsetPotential3D>& potential,
    std::vector<Cell> cells,
    const Eigen::Vector3d& x0,
    const double weight)
    : m_potential(potential)
    , m_cells(std::move(cells))
    , m_weight(weight)
    , m_c(potential ? std::max(potential->target_level(), 1e-300) : 1.)
{
    m_fixed_sum.assign(m_cells.size(), 0.);
    m_fixed_n.assign(m_cells.size(), 0);
    for (size_t k = 0; k < m_cells.size(); ++k) {
        Cell& c = m_cells[k];
        if ((c.q1 - x0).dot((c.q2 - c.q1).cross(c.q3 - c.q1)) < 0.) std::swap(c.q2, c.q3);
        for (const Eigen::Vector3d* q : {&c.q1, &c.q2, &c.q3}) {
            const double v = m_potential->value(*q);
            if (!std::isfinite(v)) continue;
            m_fixed_sum[k] += (v - m_c) / m_c;
            ++m_fixed_n[k];
        }
    }
}

void BandVolumeEnergy3D::set_corner_bound(
    std::shared_ptr<const InputTriangles> tris,
    std::vector<std::vector<int64_t>> candidates,
    const double delta)
{
    m_tris = std::move(tris);
    m_cand = std::move(candidates);
    m_delta = delta;
    m_cand_fixed.assign(m_cells.size(), {});
    for (size_t k = 0; k < m_cells.size(); ++k) {
        const Cell& c = m_cells[k];
        for (const int64_t P : m_cand[k]) {
            double s = 0.;
            for (const Eigen::Vector3d* q : {&c.q1, &c.q2, &c.q3}) {
                s += (m_tris->distance(P, *q) - m_delta) / m_delta;
            }
            m_cand_fixed[k].push_back(s);
        }
    }
}

void BandVolumeEnergy3D::set_exact_corner_bound(
    std::shared_ptr<const InputTriangles> tris,
    const double delta)
{
    m_tris = std::move(tris);
    m_delta = delta;
    m_exact = true;
    m_hint.assign(m_cells.size(), -1);
    m_min_set = false;
    m_min_md.assign(m_cells.size(), 0.);
    m_min_tri.assign(m_cells.size(), -1);
    m_ball_r = -1.;
    m_ball_cand.assign(m_cells.size(), {});
}

bool BandVolumeEnergy3D::certify_ball(const Eigen::Vector3d& x) const
{
    // r = the distance from x to the nearest plane through a cell's three fixed corners, each
    // cell's volume being positive while x stays on its side of that plane.
    double r = std::numeric_limits<double>::infinity();
    for (const Cell& c : m_cells) {
        const Eigen::Vector3d N = (c.q2 - c.q1).cross(c.q3 - c.q1);
        const double h = (c.q1 - x).dot(N) / N.norm();
        if (!(h > 0.)) return false;
        r = std::min(r, h);
    }
    if (!std::isfinite(r)) return false;
    m_ball_c = x;
    m_ball_r = r;
    for (size_t k = 0; k < m_cells.size(); ++k) {
        const Cell& c = m_cells[k];
        m_tris->ball_candidates(x, r, {{c.q1, c.q2, c.q3}}, m_ball_cand[k]);
    }
    return true;
}

void BandVolumeEnergy3D::exact_minima(const Eigen::Vector3d& x) const
{
    if (m_min_set && std::memcmp(x.data(), m_min_x.data(), 3 * sizeof(double)) == 0) return;
    const bool listed = (m_ball_r > 0. && (x - m_ball_c).norm() <= m_ball_r) || certify_ball(x);
    for (size_t k = 0; k < m_cells.size(); ++k) {
        const Cell& c = m_cells[k];
        int64_t P = -1;
        double md = std::numeric_limits<double>::infinity();
        if (listed) {
            // min_mean_distance()'s sum, order and comparison over the certified candidates.
            for (const InputTriangles::BallCandidate& cd : m_ball_cand[k]) {
                double s = 0.;
                s += m_tris->distance(cd.tri, x);
                s += cd.d1;
                s += cd.d2;
                s += cd.d3;
                s /= 4;
                if (s < md || (s == md && cd.tri < P)) md = s, P = cd.tri;
            }
        } else {
            const std::array<Eigen::Vector3d, 4> pts = {{x, c.q1, c.q2, c.q3}};
            md = m_tris->min_mean_distance(pts.data(), 4, P, m_hint[k]);
        }
        if (P >= 0) m_hint[k] = P;
        m_min_md[k] = md;
        m_min_tri[k] = P;
    }
    m_min_x = x;
    m_min_set = true;
}

double BandVolumeEnergy3D::eval(
    const Eigen::Vector3d& x,
    const int need,
    Eigen::Vector3d& g,
    Eigen::Matrix3d& H) const
{
    if (m_tris && m_exact) {
        // EXPERIMENTAL_band_volume_exact_min: per cell the min over all triangles of the corner
        // mean of r_P, the four corners being x and the cell's three fixed ones.
        exact_minima(x);
        double E = 0.;
        g.setZero();
        H.setZero();
        for (size_t k = 0; k < m_cells.size(); ++k) {
            const Cell& c = m_cells[k];
            const Eigen::Vector3d N = (c.q2 - c.q1).cross(c.q3 - c.q1);
            const double vol = (c.q1 - x).dot(N) / 6.;
            const Eigen::Vector3d gvol = -N / 6.;
            const int64_t P = m_min_tri[k];
            if (P < 0) continue;
            const double best = (m_min_md[k] - m_delta) / m_delta;
            E += vol * best;
            if (need >= 1) {
                Eigen::Vector3d gd;
                Eigen::Matrix3d hd;
                m_tris->distance(P, x, &gd, need >= 2 ? &hd : nullptr);
                const Eigen::Vector3d gm = gd / (4. * m_delta);
                g += best * gvol + vol * gm;
                if (need >= 2) {
                    H += gvol * gm.transpose() + gm * gvol.transpose() + vol * hd / (4. * m_delta);
                }
            }
        }
        E *= m_weight;
        g *= m_weight;
        H *= m_weight;
        return E;
    }
    if (m_tris) {
        // corner_bound: per cell the minimising candidate's corner mean (see set_corner_bound()).
        double E = 0.;
        g.setZero();
        H.setZero();
        for (size_t k = 0; k < m_cells.size(); ++k) {
            const Cell& c = m_cells[k];
            const Eigen::Vector3d N = (c.q2 - c.q1).cross(c.q3 - c.q1);
            const double vol = (c.q1 - x).dot(N) / 6.;
            const Eigen::Vector3d gvol = -N / 6.;
            double best = std::numeric_limits<double>::infinity();
            int64_t bestP = -1;
            for (size_t i = 0; i < m_cand[k].size(); ++i) {
                const double m =
                    (m_cand_fixed[k][i] + (m_tris->distance(m_cand[k][i], x) - m_delta) / m_delta) /
                    4.;
                if (m < best) best = m, bestP = int64_t(i);
            }
            if (bestP < 0) continue;
            E += vol * best;
            if (need >= 1) {
                Eigen::Vector3d gd;
                Eigen::Matrix3d hd;
                m_tris->distance(m_cand[k][size_t(bestP)], x, &gd, need >= 2 ? &hd : nullptr);
                const Eigen::Vector3d gm = gd / (4. * m_delta);
                g += best * gvol + vol * gm;
                if (need >= 2) {
                    H += gvol * gm.transpose() + gm * gvol.transpose() + vol * hd / (4. * m_delta);
                }
            }
        }
        E *= m_weight;
        g *= m_weight;
        H *= m_weight;
        return E;
    }
    // The moving vertex's own reading, shared by every cell.
    double vx = 0.;
    Eigen::Vector3d gx = Eigen::Vector3d::Zero();
    if (need >= 1) {
        m_potential->value_gradient(x, vx, gx);
    } else {
        vx = m_potential->value(x);
    }
    const bool x_ok = std::isfinite(vx) && (need < 1 || gx.allFinite());
    const double rx = (vx - m_c) / m_c;
    const Eigen::Vector3d drx = gx / m_c;
    Eigen::Matrix3d Hrx = Eigen::Matrix3d::Zero();
    if (need >= 2 && x_ok) {
        const Eigen::Matrix3d h = m_potential->hessian(x);
        if (h.allFinite()) Hrx = h / m_c;
    }
    double E = 0.;
    g.setZero();
    H.setZero();
    for (size_t k = 0; k < m_cells.size(); ++k) {
        const Cell& c = m_cells[k];
        const Eigen::Vector3d N = (c.q2 - c.q1).cross(c.q3 - c.q1);
        const double vol = (c.q1 - x).dot(N) / 6.;
        const Eigen::Vector3d gvol = -N / 6.;
        const Eigen::Vector3d ctr = 0.25 * (x + c.q1 + c.q2 + c.q3);
        double vc = 0.;
        Eigen::Vector3d gc = Eigen::Vector3d::Zero();
        if (need >= 1) {
            m_potential->value_gradient(ctr, vc, gc);
        } else {
            vc = m_potential->value(ctr);
        }
        const bool c_ok = std::isfinite(vc) && (need < 1 || gc.allFinite());
        double sum = m_centroid_only ? 0. : m_fixed_sum[k];
        int n = m_centroid_only ? 0 : m_fixed_n[k];
        Eigen::Vector3d gsum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d hsum = Eigen::Matrix3d::Zero();
        if (x_ok && !m_centroid_only) {
            sum += rx;
            ++n;
            gsum += drx;
            hsum += Hrx;
        }
        if (c_ok) {
            sum += (vc - m_c) / m_c;
            ++n;
            gsum += 0.25 * gc / m_c;
            if (need >= 2) {
                const Eigen::Matrix3d h = m_potential->hessian(ctr);
                if (h.allFinite()) hsum += h / (16. * m_c);
            }
        }
        if (n == 0) continue;
        const double m = sum / n;
        E += vol * m;
        if (need >= 1) {
            const Eigen::Vector3d gm = gsum / n;
            g += m * gvol + vol * gm;
            if (need >= 2) {
                H += gvol * gm.transpose() + gm * gvol.transpose() + vol * (hsum / n);
            }
        }
    }
    E *= m_weight;
    g *= m_weight;
    H *= m_weight;
    return E;
}

double BandVolumeEnergy3D::value(const TVector& xv)
{
    Eigen::Vector3d g;
    Eigen::Matrix3d H;
    return eval(xv.head(3), 0, g, H);
}

void BandVolumeEnergy3D::gradient(const TVector& xv, TVector& gradv)
{
    Eigen::Vector3d g;
    Eigen::Matrix3d H;
    eval(xv.head(3), 1, g, H);
    gradv = g;
}

void BandVolumeEnergy3D::hessian(const TVector& xv, MatrixXd& hess)
{
    Eigen::Vector3d g;
    Eigen::Matrix3d H;
    eval(xv.head(3), 2, g, H);
    hess = H;
}

// ---------------------------------------------------------------------------------------------
// StencilEnergy3D
// ---------------------------------------------------------------------------------------------

StencilEnergy3D::StencilEnergy3D(
    const std::shared_ptr<const OffsetPotential3D>& potential,
    std::vector<Face> faces,
    const double weight,
    const bool gauss_newton)
    : m_potential(potential)
    , m_faces(std::move(faces))
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
    , m_c(potential ? std::max(potential->target_level(), 1e-300) : 1.)
{}

const std::vector<StencilEnergy3D::Reading>& StencilEnergy3D::readings_at(
    const Eigen::Vector3d& x,
    const bool need_dr) const
{
    if (m_readings_valid && x == m_readings_x && (m_readings_have_dr || !need_dr)) {
        return m_readings;
    }
    m_readings.clear();
    m_readings_unscorable = false;
    for (size_t fi = 0; fi < m_faces.size(); ++fi) {
        const Face& f = m_faces[fi];
        for (size_t si = 0; si < f.samples.size(); ++si) {
            const Sample& sm = f.samples[si];
            const Eigen::Vector3d p = sm.a * x + sm.b * f.q1 + sm.c * f.q2;
            Reading rd;
            double v;
            Eigen::Vector3d g;
            if (m_reader) {
                Eigen::Matrix3d H;
                const int st = m_reader(fi, sm, x, p, v, g, H);
                if (st < 0) m_readings_unscorable = true;
                if (st > 0 && std::isfinite(v)) {
                    rd.r = (v - m_c) / m_c;
                    rd.r_ok = true;
                    if (g.allFinite()) {
                        rd.dr = g / m_c;
                        rd.dr_ok = true;
                    }
                    rd.H = H;
                }
                m_readings.push_back(rd);
                continue;
            }
            if (need_dr) {
                m_potential->value_gradient(p, v, g);
            } else {
                v = m_potential->value(p);
            }
            if (std::isfinite(v)) {
                rd.r = (v - m_c) / m_c;
                rd.r_ok = true;
                if (need_dr && g.allFinite()) {
                    rd.dr = g / m_c;
                    rd.dr_ok = true;
                }
            }
            m_readings.push_back(rd);
        }
    }
    m_readings_x = x;
    m_readings_valid = true;
    // A reader's readings carry the gradient whether or not it was asked for, so value() followed
    // by gradient() at the same x reads once (under EXPERIMENTAL_visible_distance each reading is
    // a visibility query).
    m_readings_have_dr = need_dr || bool(m_reader);
    return m_readings;
}

double StencilEnergy3D::value(const TVector& xv)
{
    const Eigen::Vector3d x = xv.head(3);
    if (m_area_weighted || m_area_integral) {
        double E;
        Eigen::Vector3d g;
        Eigen::Matrix3d H;
        area_weighted(x, 0, E, g, H);
        return m_readings_unscorable ? std::numeric_limits<double>::infinity() : E;
    }
    const std::vector<Reading>& rds = readings_at(x, false);
    if (m_readings_unscorable) return std::numeric_limits<double>::infinity();
    double E = 0.;
    size_t k = 0;
    for (const Face& f : m_faces) {
        double s = 0., nw = 0.;
        for (size_t i = 0; i < f.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok) continue;
            s += f.samples[i].w * rd.r * rd.r;
            nw += f.samples[i].w;
        }
        if (nw > 0.) E += s / nw;
    }
    return m_weight * E;
}

void StencilEnergy3D::gradient(const TVector& xv, TVector& gradv)
{
    const Eigen::Vector3d x = xv.head(3);
    if (m_area_weighted || m_area_integral) {
        double E;
        Eigen::Vector3d g;
        Eigen::Matrix3d H;
        area_weighted(x, 1, E, g, H);
        gradv = g;
        return;
    }
    const std::vector<Reading>& rds = readings_at(x, true);
    gradv = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d g = Eigen::Vector3d::Zero();
    size_t k = 0;
    for (const Face& f : m_faces) {
        // d/dx of r(q_i)^2 is 2 r dr . dq_i/dx and dq_i/dx = a_i I, so the moving vertex's own
        // barycentric weight is the whole chain rule. A corner sample of another vertex has
        // a_i = 0 and so contributes to the value but not to the gradient.
        Eigen::Vector3d gf = Eigen::Vector3d::Zero();
        double nw = 0.;
        for (size_t i = 0; i < f.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok || !rd.dr_ok) continue;
            gf += (f.samples[i].w * 2. * f.samples[i].a * rd.r) * rd.dr;
            nw += f.samples[i].w;
        }
        if (nw > 0.) g += gf / nw;
    }
    gradv = m_weight * g;
}

void StencilEnergy3D::hessian(const TVector& xv, MatrixXd& hess)
{
    const Eigen::Vector3d x = xv.head(3);
    if (m_area_weighted || m_area_integral) {
        double E;
        Eigen::Vector3d g;
        Eigen::Matrix3d H;
        area_weighted(x, 2, E, g, H);
        hess = H;
        return;
    }
    const std::vector<Reading>& rds = readings_at(x, true);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    size_t k = 0;
    for (const Face& f : m_faces) {
        // EXACT Hessian of r^2 unless gauss_newton: 2 a_i^2 (dr dr^T + r hess Phi / c). Until
        // 2026-09-28 the second term was dropped (Gauss-Newton, PSD by construction; kept as
        // gauss_newton = true, which the tests check) and 28-30% of the front solves on
        // the cube (target 1e-2, tolerance 1e-4) converged only linearly -- |grad|/|grad_0| at
        // 1e-2..1e-5 after the 10-iteration cap -- near its rounded edges, where r is still large
        // and hess Phi is the offset surface's curvature. With the term: 2% at the cap, mean 2.8
        // iterations instead of 4.6, 98% stopped on the relative gradient tolerance, smoothing
        // time unchanged (0.40 s vs 0.39 s over a 2-turn probe), placement unchanged. The term is
        // indefinite where r < 0 (inside the level set); polysolve's Newton regularises there.
        Eigen::Matrix3d Hf = Eigen::Matrix3d::Zero();
        double nw = 0.;
        for (size_t i = 0; i < f.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok || !rd.dr_ok) continue;
            const Sample& sm = f.samples[i];
            const double a = sm.a;
            Hf += (sm.w * 2. * a * a) * (rd.dr * rd.dr.transpose());
            if (!m_gauss_newton) {
                const Eigen::Vector3d p = a * x + sm.b * f.q1 + sm.c * f.q2;
                const Eigen::Matrix3d Hphi = m_reader ? rd.H : m_potential->hessian(p);
                if (Hphi.allFinite()) Hf += (sm.w * 2. * a * a * rd.r / m_c) * Hphi;
            }
            nw += sm.w;
        }
        if (nw > 0.) H += Hf / nw;
    }
    hess = m_weight * H;
}

void StencilEnergy3D::area_weighted(
    const Eigen::Vector3d& x,
    const int need,
    double& E,
    Eigen::Vector3d& g,
    Eigen::Matrix3d& H) const
{
    // E = k N / D with k = weight * n, N = sum_f area(f) O(f), D = sum_f area(f); O(f) and its
    // derivatives exactly as value(), gradient() and hessian() form them, area(f) and its
    // derivatives from n(x) = (q1 - x) x (q2 - x) = q1 x q2 + e x x, e = q2 - q1, affine in x:
    // grad area = [e]x^T u / 2 and hess area = [e]x^T (I - u u^T) [e]x / (2 |n|), u = n / |n|.
    const std::vector<Reading>& rds = readings_at(x, need >= 1);
    double N = 0., D = 0.;
    Eigen::Vector3d gN = Eigen::Vector3d::Zero(), gD = Eigen::Vector3d::Zero();
    Eigen::Matrix3d HN = Eigen::Matrix3d::Zero(), HD = Eigen::Matrix3d::Zero();
    size_t k = 0;
    for (const Face& f : m_faces) {
        double s = 0., nr = 0., ng = 0.;
        Eigen::Vector3d gf = Eigen::Vector3d::Zero();
        Eigen::Matrix3d Hf = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < f.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok) continue;
            const Sample& sm = f.samples[i];
            s += sm.w * rd.r * rd.r;
            nr += sm.w;
            if (need < 1 || !rd.dr_ok) continue;
            const double a = sm.a;
            gf += (sm.w * 2. * a * rd.r) * rd.dr;
            if (need >= 2) {
                Hf += (sm.w * 2. * a * a) * (rd.dr * rd.dr.transpose());
                if (!m_gauss_newton) {
                    const Eigen::Vector3d p = a * x + sm.b * f.q1 + sm.c * f.q2;
                    const Eigen::Matrix3d Hphi = m_reader ? rd.H : m_potential->hessian(p);
                    if (Hphi.allFinite()) Hf += (sm.w * 2. * a * a * rd.r / m_c) * Hphi;
                }
            }
            ng += sm.w;
        }
        if (!(nr > 0.)) continue;
        const double O = s / nr;
        const Eigen::Vector3d nv = (f.q1 - x).cross(f.q2 - x);
        const double nn = nv.norm();
        const double A = 0.5 * nn;
        N += A * O;
        D += A;
        if (need < 1) continue;
        const Eigen::Vector3d e = f.q2 - f.q1;
        Eigen::Matrix3d Ex;
        Ex << 0., -e.z(), e.y(), e.z(), 0., -e.x(), -e.y(), e.x(), 0.;
        Eigen::Vector3d gA = Eigen::Vector3d::Zero();
        Eigen::Matrix3d HA = Eigen::Matrix3d::Zero();
        if (nn > 0.) {
            const Eigen::Vector3d u = nv / nn;
            gA = 0.5 * (Ex.transpose() * u);
            if (need >= 2) {
                HA = (0.5 / nn) *
                     (Ex.transpose() * (Eigen::Matrix3d::Identity() - u * u.transpose()) * Ex);
            }
        }
        const Eigen::Vector3d gO = ng > 0. ? Eigen::Vector3d(gf / ng) : Eigen::Vector3d::Zero();
        gN += O * gA + A * gO;
        gD += gA;
        if (need >= 2) {
            const Eigen::Matrix3d HO = ng > 0. ? Eigen::Matrix3d(Hf / ng) : Eigen::Matrix3d::Zero();
            HN += O * HA + gA * gO.transpose() + gO * gA.transpose() + A * HO;
            HD += HA;
        }
    }
    g.setZero();
    H.setZero();
    if (m_area_integral) {
        // EXPERIMENTAL_integral_energy: the sum itself, sum_f area(f) O(f) / weight.
        E = m_weight * N;
        if (need >= 1) g = m_weight * gN;
        if (need >= 2) H = m_weight * HN;
        return;
    }
    if (!(D > 0.)) {
        E = std::numeric_limits<double>::infinity(); // every face flat: nothing to weigh by
        return;
    }
    const double kw = m_weight * double(m_faces.size());
    E = kw * N / D;
    if (need >= 1) g = kw * (gN / D - (N / (D * D)) * gD);
    if (need >= 2) {
        H = kw * (HN / D - (gN * gD.transpose() + gD * gN.transpose()) / (D * D) -
                  (N / (D * D)) * HD + (2. * N / (D * D * D)) * (gD * gD.transpose()));
    }
}

// ---------------------------------------------------------------------------------------------
// StencilEnergy2D -- StencilEnergy3D one dimension down; see there for every choice made here.
// ---------------------------------------------------------------------------------------------

StencilEnergy2D::StencilEnergy2D(
    const std::shared_ptr<const OffsetPotential2D>& potential,
    std::vector<Edge> edges,
    const double weight,
    const bool gauss_newton)
    : m_potential(potential)
    , m_edges(std::move(edges))
    , m_weight(weight)
    , m_gauss_newton(gauss_newton)
    , m_c(potential ? std::max(potential->target_level(), 1e-300) : 1.)
{}

const std::vector<StencilEnergy2D::Reading>& StencilEnergy2D::readings_at(
    const Eigen::Vector2d& x,
    const bool need_dr) const
{
    if (m_readings_valid && x == m_readings_x && (m_readings_have_dr || !need_dr)) {
        return m_readings;
    }
    m_readings.clear();
    for (const Edge& e : m_edges) {
        for (const Sample& sm : e.samples) {
            const Eigen::Vector2d p = sm.a * x + sm.b * e.q1;
            Reading rd;
            double v;
            Eigen::Vector2d g;
            if (need_dr) {
                m_potential->value_gradient(p, v, g);
            } else {
                v = m_potential->value(p);
            }
            if (std::isfinite(v)) {
                rd.r = (v - m_c) / m_c;
                rd.r_ok = true;
                if (need_dr && g.allFinite()) {
                    rd.dr = g / m_c;
                    rd.dr_ok = true;
                }
            }
            m_readings.push_back(rd);
        }
    }
    m_readings_x = x;
    m_readings_valid = true;
    m_readings_have_dr = need_dr;
    return m_readings;
}

double StencilEnergy2D::value(const TVector& xv)
{
    const Eigen::Vector2d x = xv.head(2);
    const std::vector<Reading>& rds = readings_at(x, false);
    double E = 0.;
    size_t k = 0;
    for (const Edge& e : m_edges) {
        double s = 0.;
        size_t n = 0;
        for (size_t i = 0; i < e.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok) continue;
            s += rd.r * rd.r;
            ++n;
        }
        if (n > 0) E += s / double(n);
    }
    return m_weight * E;
}

void StencilEnergy2D::gradient(const TVector& xv, TVector& gradv)
{
    const Eigen::Vector2d x = xv.head(2);
    const std::vector<Reading>& rds = readings_at(x, true);
    gradv = Eigen::VectorXd::Zero(2);
    Eigen::Vector2d g = Eigen::Vector2d::Zero();
    size_t k = 0;
    for (const Edge& e : m_edges) {
        Eigen::Vector2d ge = Eigen::Vector2d::Zero();
        size_t n = 0;
        for (size_t i = 0; i < e.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok || !rd.dr_ok) continue;
            ge += (2. * e.samples[i].a * rd.r) * rd.dr;
            ++n;
        }
        if (n > 0) g += ge / double(n);
    }
    gradv = m_weight * g;
}

void StencilEnergy2D::hessian(const TVector& xv, MatrixXd& hess)
{
    const Eigen::Vector2d x = xv.head(2);
    const std::vector<Reading>& rds = readings_at(x, true);
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    size_t k = 0;
    for (const Edge& e : m_edges) {
        Eigen::Matrix2d He = Eigen::Matrix2d::Zero();
        size_t n = 0;
        for (size_t i = 0; i < e.samples.size(); ++i, ++k) {
            const Reading& rd = rds[k];
            if (!rd.r_ok || !rd.dr_ok) continue;
            const Sample& sm = e.samples[i];
            const double a = sm.a;
            He += (2. * a * a) * (rd.dr * rd.dr.transpose());
            if (!m_gauss_newton) {
                const Eigen::Vector2d p = a * x + sm.b * e.q1;
                const Eigen::Matrix2d Hphi = m_potential->hessian(p);
                if (Hphi.allFinite()) He += (2. * a * a * rd.r / m_c) * Hphi;
            }
            ++n;
        }
        if (n > 0) H += He / double(n);
    }
    hess = m_weight * H;
}

} // namespace wmtk::components::topological_offset
