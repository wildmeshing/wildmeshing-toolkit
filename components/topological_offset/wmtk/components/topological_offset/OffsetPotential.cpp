#include "OffsetPotential.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>

#include <Eigen/Eigenvalues>

#include <ipc/collision_mesh.hpp>
#include <ipc/esp/arbitrary_point_esp.hpp>
#include <ipc/esp/esp_parameters.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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
    // Zero contributes no offset force; a backstop, since no front vertex sits on the complex.
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

// ---------------------------------------------------------------------------------------------
// InputTriangles
// ---------------------------------------------------------------------------------------------

namespace {
/// The distance from p to triangle (a, b, c), with InputTriangles::distance()'s derivatives.
double triangle_distance(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& p,
    Eigen::Vector3d* grad,
    Eigen::Matrix3d* hess)
{
    // The closest point and the feature it lies on (Ericson, Real-Time Collision Detection 5.1.5).
    const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
    Eigen::Vector3d foot, edge = Eigen::Vector3d::Zero();
    int kind = 2; // 0 vertex, 1 edge (direction `edge`), 2 interior
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
} // namespace

// ---------------------------------------------------------------------------------------------

InputTriangles::InputTriangles(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
    : m_V(V)
    , m_F(F)
{
    m_bvh.init(m_V, m_F, 1e-6);
}

void InputTriangles::near_triangles(
    const Eigen::Vector3d& p,
    std::vector<std::pair<int64_t, double>>& out) const
{
    // SimpleBVH's nearest_facet returns its own (reordered) facet index; its box query returns
    // F's. So: the nearest distance from the BVH, then every triangle whose box reaches within it.
    out.clear();
    Eigen::Vector3d q;
    double sq = 0.;
    m_bvh.nearest_facet(p, q, sq);
    const double r = std::sqrt(sq) * (1. + 1e-12) + 1e-12 + m_tol;
    std::vector<unsigned int> list;
    m_bvh.intersect_box(p - Eigen::Vector3d::Constant(r), p + Eigen::Vector3d::Constant(r), list);
    for (const unsigned int t : list) {
        const Eigen::Vector3d a = m_V.row(m_F(t, 0)).head<3>().transpose();
        const Eigen::Vector3d b = m_V.row(m_F(t, 1)).head<3>().transpose();
        const Eigen::Vector3d c = m_V.row(m_F(t, 2)).head<3>().transpose();
        out.emplace_back(int64_t(t), triangle_distance(a, b, c, p, nullptr, nullptr));
    }
}

int64_t InputTriangles::nearest(const Eigen::Vector3d& p) const
{
    // The closest triangle (pieces mode: the piece of the closest boundary triangle) -- the
    // lowest index on a tie.
    std::vector<std::pair<int64_t, double>> near;
    near_triangles(p, near);
    int64_t best = -1;
    double bd = std::numeric_limits<double>::infinity();
    for (const auto& [t, d] : near) {
        const int64_t id = m_pieces ? m_tri_piece[size_t(t)] : t;
        if (d < bd || (d == bd && id < best)) bd = d, best = id;
    }
    return best;
}

void InputTriangles::nearest_all(const Eigen::Vector3d& p, std::vector<int64_t>& out) const
{
    if (!m_pieces) {
        out.push_back(nearest(p));
        return;
    }
    std::vector<std::pair<int64_t, double>> near;
    near_triangles(p, near);
    double bd = std::numeric_limits<double>::infinity();
    for (const auto& tp : near) bd = std::min(bd, tp.second);
    for (const auto& [t, d] : near) {
        if (d <= bd + m_tol) out.push_back(m_tri_piece[size_t(t)]);
    }
}

double InputTriangles::distance(
    const int64_t tri,
    const Eigen::Vector3d& p,
    Eigen::Vector3d* grad,
    Eigen::Matrix3d* hess) const
{
    const auto corner = [&](const int64_t r, const int j) -> Eigen::Vector3d {
        return m_V.row(m_F(r, j)).head<3>().transpose();
    };
    if (!m_pieces)
        return triangle_distance(corner(tri, 0), corner(tri, 1), corner(tri, 2), p, grad, hess);
    // A convex piece: 0 inside (every outward plane on the inner side), else the nearest of its
    // boundary triangles, whose foot and feature are the piece's.
    bool inside = true;
    for (const Eigen::Vector4d& pl : m_piece_planes[size_t(tri)]) {
        if (pl.head<3>().dot(p) - pl[3] > m_tol) {
            inside = false;
            break;
        }
    }
    if (inside) {
        if (grad) grad->setZero();
        if (hess) hess->setZero();
        return 0.;
    }
    double bd = std::numeric_limits<double>::infinity();
    int64_t br = -1;
    for (const int64_t r : m_piece_tris[size_t(tri)]) {
        const double d =
            triangle_distance(corner(r, 0), corner(r, 1), corner(r, 2), p, nullptr, nullptr);
        if (d < bd) bd = d, br = r;
    }
    return triangle_distance(corner(br, 0), corner(br, 1), corner(br, 2), p, grad, hess);
}

std::shared_ptr<InputTriangles> InputTriangles::convex_pieces(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T)
{
    using Key = std::array<int, 3>;
    const auto key = [](int a, int b, int c) {
        Key k{{a, b, c}};
        std::sort(k.begin(), k.end());
        return k;
    };
    const auto pos = [&](const int v) -> Eigen::Vector3d { return V.row(v).head<3>().transpose(); };
    double diag = 0.;
    if (V.rows() > 0) diag = (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();
    const double tol = 1e-9 * std::max(diag, 1e-300);

    // A piece: its tets, its vertices, its volume, and its boundary faces oriented outward.
    struct Piece
    {
        std::vector<int> tets;
        std::set<int> verts;
        std::map<Key, std::array<int, 3>> boundary;
        double volume = 0.;
        bool alive = true;
    };
    std::vector<Piece> pieces(size_t(T.rows()));
    for (int t = 0; t < T.rows(); ++t) {
        Piece& pc = pieces[size_t(t)];
        pc.tets.push_back(t);
        Eigen::Vector3d ctr = Eigen::Vector3d::Zero();
        for (int j = 0; j < 4; ++j) pc.verts.insert(T(t, j)), ctr += pos(T(t, j)) / 4.;
        pc.volume =
            std::abs((pos(T(t, 1)) - pos(T(t, 0)))
                         .dot((pos(T(t, 2)) - pos(T(t, 0))).cross(pos(T(t, 3)) - pos(T(t, 0))))) /
            6.;
        for (int j = 0; j < 4; ++j) {
            std::array<int, 3> f{{T(t, (j + 1) % 4), T(t, (j + 2) % 4), T(t, (j + 3) % 4)}};
            const Eigen::Vector3d n = (pos(f[1]) - pos(f[0])).cross(pos(f[2]) - pos(f[0]));
            if (n.dot(ctr - pos(f[0])) > 0.) std::swap(f[1], f[2]); // outward
            pc.boundary[key(f[0], f[1], f[2])] = f;
        }
    }
    const auto plane = [&](const std::array<int, 3>& f) {
        Eigen::Vector3d n = (pos(f[1]) - pos(f[0])).cross(pos(f[2]) - pos(f[0]));
        n.normalize();
        return Eigen::Vector4d(n.x(), n.y(), n.z(), n.dot(pos(f[0])));
    };
    // The union of face-connected pieces is convex iff every one of its boundary faces has all of
    // its vertices on the inner side of the face's plane. Its boundary is the faces carried by
    // exactly one of the pieces (a face shared by two is interior).
    const auto union_convex = [&](const std::vector<int>& ids) {
        std::map<Key, std::pair<std::array<int, 3>, int>> faces;
        std::set<int> verts;
        for (const int i : ids) {
            for (const auto& [k, f] : pieces[size_t(i)].boundary) {
                auto [it, fresh] = faces.emplace(k, std::make_pair(f, 0));
                ++it->second.second;
            }
            verts.insert(pieces[size_t(i)].verts.begin(), pieces[size_t(i)].verts.end());
        }
        size_t shared = 0;
        for (const auto& [k, fc] : faces) {
            if (fc.second > 1) {
                ++shared;
                continue;
            }
            const Eigen::Vector4d pl = plane(fc.first);
            for (const int v : verts) {
                if (pl.head<3>().dot(pos(v)) - pl[3] > tol) return false;
            }
        }
        return shared > 0;
    };
    const auto merge = [&](const std::vector<int>& ids) {
        Piece& A = pieces[size_t(ids[0])];
        for (size_t n = 1; n < ids.size(); ++n) {
            Piece& B = pieces[size_t(ids[n])];
            for (const auto& [k, f] : B.boundary) {
                if (!A.boundary.erase(k)) A.boundary[k] = f;
            }
            A.tets.insert(A.tets.end(), B.tets.begin(), B.tets.end());
            A.verts.insert(B.verts.begin(), B.verts.end());
            A.volume += B.volume;
            B = Piece();
            B.alive = false;
        }
    };
    // The pieces adjacent to each piece, from the faces two live pieces share.
    const auto adjacency = [&]() {
        std::map<Key, std::vector<int>> owner;
        for (size_t i = 0; i < pieces.size(); ++i) {
            if (!pieces[i].alive) continue;
            for (const auto& [k, f] : pieces[i].boundary) owner[k].push_back(int(i));
        }
        std::map<int, std::set<int>> adj;
        for (const auto& [k, o] : owner) {
            if (o.size() == 2) adj[o[0]].insert(o[1]), adj[o[1]].insert(o[0]);
        }
        return adj;
    };
    // Best first: the convex merge of 2 pieces with the largest union volume; when none is left,
    // of 3 (a piece and two of its neighbours) -- a cube of Kuhn tets is two 180-degree halves,
    // which no chain of pairwise convex merges need reach.
    for (;;) {
        const auto adj = adjacency();
        double best = -1.;
        std::vector<int> pick;
        for (const auto& [a, nb] : adj) {
            for (const int b : nb) {
                if (b < a) continue;
                const double v = pieces[size_t(a)].volume + pieces[size_t(b)].volume;
                if (v > best && union_convex({a, b})) best = v, pick = {a, b};
            }
        }
        if (pick.empty()) {
            for (const auto& [a, nb] : adj) {
                const std::vector<int> n(nb.begin(), nb.end());
                for (size_t i = 0; i < n.size(); ++i) {
                    for (size_t j = i + 1; j < n.size(); ++j) {
                        const double v = pieces[size_t(a)].volume + pieces[size_t(n[i])].volume +
                                         pieces[size_t(n[j])].volume;
                        if (v > best && union_convex({a, n[i], n[j]}))
                            best = v, pick = {a, n[i], n[j]};
                    }
                }
            }
        }
        if (pick.empty()) break;
        merge(pick);
    }

    auto out = std::shared_ptr<InputTriangles>(new InputTriangles());
    out->m_pieces = true;
    out->m_tol = tol;
    out->m_V = V.leftCols(3);
    std::vector<std::array<int, 3>> rows;
    for (const Piece& pc : pieces) {
        if (!pc.alive) continue;
        const int64_t id = int64_t(out->m_piece_tris.size());
        out->m_piece_tris.emplace_back();
        out->m_piece_planes.emplace_back();
        out->m_piece_tet_counts.push_back(pc.tets.size());
        for (const auto& [k, f] : pc.boundary) {
            out->m_piece_tris.back().push_back(int64_t(rows.size()));
            out->m_tri_piece.push_back(id);
            out->m_piece_planes.back().push_back(plane(f));
            rows.push_back(f);
        }
    }
    out->m_F.resize(Eigen::Index(rows.size()), 3);
    for (size_t r = 0; r < rows.size(); ++r) {
        for (int j = 0; j < 3; ++j) out->m_F(Eigen::Index(r), j) = rows[r][size_t(j)];
    }
    out->m_bvh.init(out->m_V, out->m_F, 1e-6);
    return out;
}

// ---------------------------------------------------------------------------------------------
// InputSegments
// ---------------------------------------------------------------------------------------------

InputSegments::InputSegments(const Eigen::MatrixXd& V, const Eigen::MatrixXi& E)
    : m_E(E)
{
    m_V = Eigen::MatrixXd::Zero(V.rows(), 3);
    m_V.leftCols(std::min<Eigen::Index>(2, V.cols())) =
        V.leftCols(std::min<Eigen::Index>(2, V.cols()));
    m_bvh.init(m_V, m_E, 1e-6);
}

int64_t InputSegments::nearest(const Eigen::Vector2d& p) const
{
    // As InputTriangles::nearest(): the nearest distance from the BVH, then every segment whose box
    // reaches within it, the closest of those by distance() (lowest index on a tie).
    const Eigen::Vector3d p3(p.x(), p.y(), 0.);
    Eigen::Vector3d q;
    double sq = 0.;
    m_bvh.nearest_facet(p3, q, sq);
    const double r = std::sqrt(sq) * (1. + 1e-12) + 1e-12;
    std::vector<unsigned int> list;
    m_bvh.intersect_box(p3 - Eigen::Vector3d::Constant(r), p3 + Eigen::Vector3d::Constant(r), list);
    int64_t best = -1;
    double bd = std::numeric_limits<double>::infinity();
    for (const unsigned int e : list) {
        const double d = distance(int64_t(e), p);
        if (d < bd || (d == bd && int64_t(e) < best)) bd = d, best = int64_t(e);
    }
    return best;
}

double InputSegments::distance(
    const int64_t seg,
    const Eigen::Vector2d& p,
    Eigen::Vector2d* grad,
    Eigen::Matrix2d* hess) const
{
    const Eigen::Vector2d a = m_V.row(m_E(seg, 0)).head<2>().transpose();
    const Eigen::Vector2d b = m_V.row(m_E(seg, 1)).head<2>().transpose();
    const Eigen::Vector2d ab = b - a;
    const double L2 = ab.squaredNorm();
    double t = L2 > 0. ? (p - a).dot(ab) / L2 : 0.;
    bool at_vertex = true;
    if (t <= 0.) {
        t = 0.;
    } else if (t >= 1.) {
        t = 1.;
    } else {
        at_vertex = false;
    }
    const Eigen::Vector2d foot = a + t * ab;
    const double d = (p - foot).norm();
    if (grad) *grad = d > 0. ? Eigen::Vector2d((p - foot) / d) : Eigen::Vector2d::Zero();
    if (hess) {
        hess->setZero();
        // The distance to a line is affine across it: no curvature in a segment's interior.
        if (d > 0. && at_vertex) {
            const Eigen::Vector2d u = (p - foot) / d;
            *hess = (Eigen::Matrix2d::Identity() - u * u.transpose()) / d;
        }
    }
    return d;
}

// ---------------------------------------------------------------------------------------------
// VolAMIPSEnergy
// ---------------------------------------------------------------------------------------------

namespace {
constexpr double factorial(const int n)
{
    return n <= 1 ? 1. : double(n) * factorial(n - 1);
}

/// det(E) and its gradient in x, E's columns q_k - x.
template <int DIM>
void det_and_grad(
    const Eigen::Matrix<double, DIM, DIM>& E,
    double& g,
    Eigen::Matrix<double, DIM, 1>& dg)
{
    if constexpr (DIM == 3) {
        const Eigen::Vector3d e0 = E.col(0), e1 = E.col(1), e2 = E.col(2);
        g = e0.dot(e1.cross(e2));
        // d det / d e_k is the k-th cofactor column, and every column moves as -x.
        dg = -(e1.cross(e2) + e2.cross(e0) + e0.cross(e1));
    } else {
        g = E(0, 0) * E(1, 1) - E(1, 0) * E(0, 1);
        const Eigen::Vector2d d0(E(1, 1), -E(0, 1)); // d g / d e0
        const Eigen::Vector2d d1(-E(1, 0), E(0, 0)); // d g / d e1
        dg = -(d0 + d1);
    }
}
} // namespace

template <int DIM>
typename VolAMIPSEnergy<DIM>::MatD VolAMIPSEnergy<DIM>::regular_rest()
{
    MatD R;
    if constexpr (DIM == 3) {
        R.col(0) = Eigen::Vector3d(1., 0., 0.);
        R.col(1) = Eigen::Vector3d(0.5, std::sqrt(3.) / 2., 0.);
        R.col(2) = Eigen::Vector3d(0.5, std::sqrt(3.) / 6., std::sqrt(2. / 3.));
    } else {
        R.col(0) = Eigen::Vector2d(1., 0.);
        R.col(1) = Eigen::Vector2d(0.5, std::sqrt(3.) / 2.);
    }
    return R;
}

template <int DIM>
bool VolAMIPSEnergy<DIM>::cell(const std::array<VecD, DIM>& q, const MatD& R, Cell& out)
{
    const double detR = R.determinant();
    if (!(detR > 0.)) return false;
    out.q = q;
    out.rest_inv = R.inverse();
    out.s = out.rest_inv.transpose() * VecD::Ones();
    out.c = detR * detR / factorial(DIM);
    return true;
}

template <int DIM>
double VolAMIPSEnergy<DIM>::value_of(const std::array<VecD, DIM + 1>& p, const MatD& R)
{
    const double detR = R.determinant();
    if (!(detR > 0.)) return std::numeric_limits<double>::infinity();
    MatD E;
    for (int k = 0; k < DIM; ++k) E.col(k) = p[size_t(k + 1)] - p[0];
    const double g = E.determinant();
    if (!(g > 0.)) return std::numeric_limits<double>::infinity();
    const double f = (E * R.inverse()).squaredNorm();
    const double v = std::pow(f, DIM) * detR * detR / (factorial(DIM) * g);
    return std::isfinite(v) ? v : std::numeric_limits<double>::infinity();
}

template <int DIM>
VolAMIPSEnergy<DIM>::VolAMIPSEnergy(std::vector<Cell> cells, const double weight)
    : m_cells(std::move(cells))
    , m_weight(weight)
{}

template <int DIM>
double VolAMIPSEnergy<DIM>::eval(const VecD& x, const int need, VecD& gr, MatD& H) const
{
    constexpr int k = DIM;
    double E = 0.;
    gr.setZero();
    H.setZero();
    for (const Cell& c : m_cells) {
        MatD Em;
        for (int j = 0; j < DIM; ++j) Em.col(j) = c.q[size_t(j)] - x;
        double g;
        VecD dg;
        det_and_grad<DIM>(Em, g, dg);
        if (!(g > 0.)) return std::numeric_limits<double>::infinity();
        const MatD F = Em * c.rest_inv;
        const double f = F.squaredNorm();
        const double fk = std::pow(f, k);
        E += c.c * fk / g;
        if (need < 1) continue;
        const VecD df = -2. * F * c.s;
        const double fk1 = std::pow(f, k - 1);
        gr += c.c * (k * fk1 * df / g - fk * dg / (g * g));
        if (need < 2) continue;
        const double fk2 = std::pow(f, k - 2);
        const MatD Hf = 2. * c.s.squaredNorm() * MatD::Identity();
        H += c.c * (k * (k - 1) * fk2 * (df * df.transpose()) / g + k * fk1 * Hf / g -
                    k * fk1 * (df * dg.transpose() + dg * df.transpose()) / (g * g) +
                    2. * fk * (dg * dg.transpose()) / (g * g * g));
    }
    E *= m_weight;
    gr *= m_weight;
    H *= m_weight;
    return std::isfinite(E) ? E : std::numeric_limits<double>::infinity();
}

template <int DIM>
double VolAMIPSEnergy<DIM>::value(const TVector& xv)
{
    VecD g;
    MatD H;
    return eval(xv.head(DIM), 0, g, H);
}

template <int DIM>
void VolAMIPSEnergy<DIM>::gradient(const TVector& xv, TVector& gradv)
{
    VecD g;
    MatD H;
    eval(xv.head(DIM), 1, g, H);
    gradv = g;
}

template <int DIM>
void VolAMIPSEnergy<DIM>::hessian(const TVector& xv, MatrixXd& hess)
{
    VecD g;
    MatD H;
    eval(xv.head(DIM), 2, g, H);
    hess = H;
}

template class VolAMIPSEnergy<2>;
template class VolAMIPSEnergy<3>;

// ---------------------------------------------------------------------------------------------
// BandVolumeEnergy
// ---------------------------------------------------------------------------------------------

template <int DIM>
BandVolumeEnergy<DIM>::BandVolumeEnergy(
    std::shared_ptr<const InputPrimitives<DIM>> prims,
    std::vector<Cell> cells,
    const double delta,
    const double weight)
    : m_prims(std::move(prims))
    , m_cells(std::move(cells))
    , m_delta(delta)
    , m_weight(weight)
{
    m_fixed.assign(m_cells.size(), {});
    for (size_t k = 0; k < m_cells.size(); ++k) {
        for (const int64_t P : m_cells[k].candidates) {
            double sum = 0.;
            for (const VecD& q : m_cells[k].q) sum += (m_prims->distance(P, q) - m_delta) / m_delta;
            m_fixed[k].push_back(sum);
        }
    }
}

template <int DIM>
double BandVolumeEnergy<DIM>::eval(const VecD& x, const int need, VecD& gr, MatD& H) const
{
    constexpr double nc = double(DIM + 1);
    double E = 0.;
    gr.setZero();
    H.setZero();
    for (size_t k = 0; k < m_cells.size(); ++k) {
        const Cell& c = m_cells[k];
        MatD Em;
        for (int j = 0; j < DIM; ++j) Em.col(j) = c.q[size_t(j)] - x;
        double g;
        VecD dg;
        det_and_grad<DIM>(Em, g, dg);
        const double vol = g / factorial(DIM);
        const VecD gvol = dg / factorial(DIM);
        double best = std::numeric_limits<double>::infinity();
        int64_t best_i = -1;
        for (size_t i = 0; i < c.candidates.size(); ++i) {
            const double m =
                (m_fixed[k][i] + (m_prims->distance(c.candidates[i], x) - m_delta) / m_delta) / nc;
            if (m < best) best = m, best_i = int64_t(i);
        }
        if (best_i < 0) continue;
        E += vol * best;
        if (need < 1) continue;
        VecD gd;
        MatD hd;
        m_prims->distance(c.candidates[size_t(best_i)], x, &gd, need >= 2 ? &hd : nullptr);
        const VecD gm = gd / (nc * m_delta);
        gr += best * gvol + vol * gm;
        if (need >= 2) {
            H += gvol * gm.transpose() + gm * gvol.transpose() + vol * hd / (nc * m_delta);
        }
    }
    E *= m_weight;
    gr *= m_weight;
    H *= m_weight;
    return E;
}

template <int DIM>
double BandVolumeEnergy<DIM>::value(const TVector& xv)
{
    VecD g;
    MatD H;
    return eval(xv.head(DIM), 0, g, H);
}

template <int DIM>
void BandVolumeEnergy<DIM>::gradient(const TVector& xv, TVector& gradv)
{
    VecD g;
    MatD H;
    eval(xv.head(DIM), 1, g, H);
    gradv = g;
}

template <int DIM>
void BandVolumeEnergy<DIM>::hessian(const TVector& xv, MatrixXd& hess)
{
    VecD g;
    MatD H;
    eval(xv.head(DIM), 2, g, H);
    hess = H;
}

template class BandVolumeEnergy<2>;
template class BandVolumeEnergy<3>;

} // namespace wmtk::components::topological_offset
