#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <wmtk/components/topological_offset/ExactBandIntegral2D.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>
#include <wmtk/utils/predicates.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::topological_offset;

/**
 * EXPERIMENTAL_band_volume_rule "exact_swept" in 2D (TopoOffsetTriMesh, EXACT SWEPT CHANGE), the
 * twins of test_exact_swept.cpp: band_swept_change() against the change of the sum of value()
 * over the band faces, the search model (BandVolumeEnergy2D::set_swept()) against
 * band_swept_change(), central differences and the "exact" rule's derivatives, and the rules that
 * read them (swept_accept(), the collapse rule).
 */

namespace {

using Vec2 = Eigen::Vector2d;
using Tri = std::array<Vec2, 3>;
constexpr double kDelta = 0.1;

/// The square [-1, 1]^2, counter-clockwise, every segment's normal out of the solid, the band
/// outside (the 2D cube).
std::shared_ptr<const ExactBandIntegral2D> square()
{
    Eigen::MatrixXd V(4, 2);
    V << -1, -1, 1, -1, 1, 1, -1, 1;
    Eigen::MatrixXi E(4, 2);
    E << 0, 1, 1, 2, 2, 3, 3, 0;
    return std::make_shared<ExactBandIntegral2D>(
        V,
        E,
        std::vector<int>{},
        std::vector<int8_t>(4, 1),
        1);
}

/// The valley: A on the x axis, B on the y axis, both normals into x, y > 0 (the offset side);
/// there d = min(x, y), and the bisector x = y is a line (two edges meeting).
std::shared_ptr<const ExactBandIntegral2D> valley()
{
    Eigen::MatrixXd V(3, 2);
    V << 10, 0, 0, 0, 0, 10;
    Eigen::MatrixXi E(2, 2);
    E << 0, 1, 1, 2;
    return std::make_shared<ExactBandIntegral2D>(
        V,
        E,
        std::vector<int>{},
        std::vector<int8_t>(2, 1),
        1);
}

/// A segment on y = 0 and an isolated point (0, 1), no side: their bisector 2 y = x^2 + 1 is a
/// parabola, and every piece with both members is rule (a)'s.
std::shared_ptr<const ExactBandIntegral2D> segment_and_point()
{
    Eigen::MatrixXd V(3, 2);
    V << -5, 0, 5, 0, 0, 1;
    Eigen::MatrixXi E(1, 2);
    E << 0, 1;
    return std::make_shared<ExactBandIntegral2D>(V, E, std::vector<int>{2});
}

/// A mesh of the faces F on the points pts, face i labelled L[i], every edge untracked and no
/// vertex held, the band term read from eb (the 2D twin of test_exact_swept.cpp's make_mesh()).
std::unique_ptr<TopoOffsetTriMesh> make_mesh(
    Parameters& param,
    const std::vector<Vec2>& pts,
    const std::vector<std::array<int, 3>>& F,
    const std::vector<int>& L,
    const std::shared_ptr<const ExactBandIntegral2D>& eb,
    const std::string& rule = "exact_swept")
{
    param.target_distance = kDelta;
    param.front_conv = 1e-3;
    param.stencil_order = 1;
    param.envelope_size = 1e-3; // tests skip Parameters::init()
    param.band_volume_rule = rule;
    Eigen::MatrixXd V(int(pts.size()), 2);
    for (size_t i = 0; i < pts.size(); ++i) V.row(int(i)) = pts[i].transpose();
    Eigen::MatrixXi Fm(int(F.size()), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        std::array<int, 3> f = F[i];
        const Vec2 a = pts[size_t(f[0])], b = pts[size_t(f[1])], c = pts[size_t(f[2])];
        if ((b - a).x() * (c - a).y() - (b - a).y() * (c - a).x() < 0.) std::swap(f[1], f[2]);
        for (int j = 0; j < 3; ++j) Fm(int(i), j) = f[size_t(j)];
    }
    auto mesh = std::make_unique<TopoOffsetTriMesh>(param, 0);
    MatrixSi Tags(int(F.size()), 1);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh->init_from_image(V, Fm, Tags, V_env, F_env, {"a"});
    for (const auto& e : mesh->get_edges()) {
        mesh->m_edge_attribute[e.eid(*mesh)].m_is_surface_fs = false;
    }
    for (size_t v = 0; v < pts.size(); ++v) {
        mesh->m_vertex_attribute[v].m_is_on_surface = false;
        mesh->m_vertex_extra[v].m_is_on_region = false;
        mesh->m_vertex_extra[v].m_boundary_mask = 0;
    }
    for (size_t i = 0; i < F.size(); ++i) {
        REQUIRE(!mesh->is_inverted(i));
        mesh->m_face_extra[i].label = L[i];
    }
    mesh->m_band_exact = eb;
    return mesh;
}

/// A square of half-width w around c, its 4 sides split in 2 segments each and coned to m (the 2D
/// twin of test_exact_swept.cpp's box()): the points (m, then the 8 on the boundary
/// counter-clockwise from the corner (-w, -w)) and 8 faces, face i over boundary segment i.
void box(
    const Vec2& c,
    const double w,
    const Vec2& m,
    std::vector<Vec2>& pts,
    std::vector<std::array<int, 3>>& F)
{
    const double r[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}};
    pts.assign(1, m);
    for (const auto& q : r) pts.push_back(c + w * Vec2(q[0], q[1]));
    F.clear();
    for (int i = 1; i <= 8; ++i) F.push_back({{0, i, i % 8 + 1}});
}

Tri face_pts(const TopoOffsetTriMesh& m, const size_t fid)
{
    const auto v = m.oriented_tri_vids(fid);
    return {
        {m.m_vertex_attribute[v[0]].m_posf,
         m.m_vertex_attribute[v[1]].m_posf,
         m.m_vertex_attribute[v[2]].m_posf}};
}

/// The sum of value() over the band faces, over delta (E* / delta).
double band_sum(
    const TopoOffsetTriMesh& m,
    const ExactBandIntegral2D& eb,
    const size_t n,
    ExactBandIntegral2D::Stats* st = nullptr)
{
    double s = 0.;
    for (size_t fid = 0; fid < n; ++fid) {
        if (m.m_face_extra[fid].label != 2) continue;
        s += eb.value(face_pts(m, fid), kDelta, st);
    }
    return s / kDelta;
}

struct Fixture
{
    const char* name;
    std::shared_ptr<const ExactBandIntegral2D> eb;
    Vec2 c;
    double w;
};

} // namespace

TEST_CASE("exact-swept-identities-2d", "[offset][exact_swept][2d]")
{
    // The 2D twin of exact-swept-identities (TopoOffsetTriMesh, EXACT SWEPT CHANGE):
    //  (1) a vertex move: band_swept_change() against the change of the sum of value() over the
    //      band faces, on a square coned to the moving vertex with random labels (band,
    //      background, input), random moves inside it -- mixed signs over the boundary edges --
    //      at the square's corner, across its side (the offset side clipped) and across the
    //      valley's bisector, each at three scales of the square (w / delta = 0.04, 0.4, 4: the
    //      largest crosses the input), and with rule (a) present (segment_and_point()), where the
    //      swept change is not the difference of the faces' bounds: there their gap is bounded by
    //      the rule-(a) area times the distance bound, over delta;
    //  (2) a collapse: the same with the vertex moved onto a corner of the square, the faces
    //      through it flat;
    //  (3) a swap: 2D's only swap, the interior flip, has swept change 0: the band sums of the
    //      flipped and the unflipped quad agree;
    //  (4) value() does not depend on the corner order.
    std::printf("exact-swept-identities-2d\n");
    std::mt19937 rng(20261111);
    std::uniform_real_distribution<double> U(-1., 1.);
    const auto sq = square();
    const auto va = valley();
    const auto sp = segment_and_point();
    std::vector<Fixture> fixtures;
    for (const double w : {0.004, 0.04, 0.4}) {
        fixtures.push_back({"square corner", sq, Vec2(1.04, 1.03), w});
        fixtures.push_back({"square side", sq, Vec2(1.0, 0.3), w});
        fixtures.push_back({"valley bisector", va, Vec2(0.05, 0.052), w});
    }
    fixtures.push_back({"rule (a)", sp, Vec2(0.5, 0.6), 0.08});
    for (const Fixture& fx : fixtures) {
        const bool rule_a_case = fx.eb == sp;
        double worst_move = 0., worst_collapse = 0., scale = 0., worst_ratio = 0.;
        double rule_a_area = 0.;
        int mixed = 0, n_moves = 0, clipped = 0;
        for (int trial = 0; trial < 40; ++trial) {
            std::vector<Vec2> pts;
            std::vector<std::array<int, 3>> F;
            const Vec2 m0 = fx.c + 0.3 * fx.w * Vec2(U(rng), U(rng));
            box(fx.c, fx.w, m0, pts, F);
            std::vector<int> L(F.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            Parameters param;
            auto mesh = make_mesh(param, pts, F, L, fx.eb);
            const size_t n = F.size();
            ExactBandIntegral2D::Stats st;
            const double before = band_sum(*mesh, *fx.eb, n, &st);
            // (1) a move inside the square
            const Vec2 x0 = m0, x1 = fx.c + 0.6 * fx.w * Vec2(U(rng), U(rng));
            const double swept = mesh->band_swept_change(0, x0, x1, &st);
            mesh->m_vertex_attribute[0].m_posf = x1;
            const double after = band_sum(*mesh, *fx.eb, n, &st);
            mesh->m_vertex_attribute[0].m_posf = x0;
            scale = std::max(scale, std::abs(before));
            const double gap = std::abs(swept - (after - before));
            worst_move = std::max(worst_move, gap);
            clipped += st.side_clipped > 0;
            if (rule_a_case) {
                // each rule-(a) piece overshoots the integral of d by at most its area times
                // the largest member distance on it, here below 2 (the box lies within 1.1 of
                // both members)
                rule_a_area += st.rule_a_area;
                if (st.rule_a_area > 0.) {
                    worst_ratio = std::max(worst_ratio, gap / (2. * st.rule_a_area / kDelta));
                }
            }
            ++n_moves;
            {
                // mixed: boundary edges of B through the vertex swept both ways
                std::vector<BandVolumeEnergy2D::SweptEdge> edges;
                mesh->band_swept_edges(0, x0, edges);
                int pos = 0, neg = 0;
                for (const auto& f : edges) {
                    const auto o = f.x_first ? wmtk::utils::predicates::orient2d(x0, f.a, x1)
                                             : wmtk::utils::predicates::orient2d(f.a, x0, x1);
                    if (o == wmtk::utils::predicates::Orientation::NEGATIVE) ++pos;
                    if (o == wmtk::utils::predicates::Orientation::POSITIVE) ++neg;
                }
                if (pos > 0 && neg > 0) ++mixed;
            }
            // (2) a collapse of the vertex onto the square's corner (point 1)
            if (!rule_a_case) {
                const Vec2 x2 = mesh->m_vertex_attribute[1].m_posf;
                const double sw = mesh->band_swept_change(0, x0, x2);
                double old_sum = 0., new_sum = 0.;
                for (size_t fid = 0; fid < n; ++fid) {
                    if (L[fid] != 2) continue;
                    const auto v = mesh->oriented_tri_vids(fid);
                    old_sum += fx.eb->value(face_pts(*mesh, fid), kDelta);
                    if (std::find(v.begin(), v.end(), size_t(1)) != v.end()) continue; // flat
                    Tri p = face_pts(*mesh, fid);
                    for (int k = 0; k < 3; ++k) {
                        if (v[size_t(k)] == 0) p[size_t(k)] = x2;
                    }
                    new_sum += fx.eb->value(p, kDelta);
                }
                worst_collapse =
                    std::max(worst_collapse, std::abs(sw - (new_sum - old_sum) / kDelta));
            }
        }
        INFO(fx.name << ", w " << fx.w);
        CHECK(mixed > 0);
        if (rule_a_case) {
            CHECK(rule_a_area > 0.);
            CHECK(worst_ratio <= 1.);
            std::printf(
                "  (1) %s, w %g: %d moves (%d mixed): swept against the difference of the sums "
                "%.1e (band term scale %.1e), at most %.2g of the rule-(a) bound\n",
                fx.name,
                fx.w,
                n_moves,
                mixed,
                worst_move,
                scale,
                worst_ratio);
            continue;
        }
        // in units of the band term, against the square's band term scale
        CHECK(worst_move <= 1e-13 * std::max(scale, 1e-6));
        CHECK(worst_collapse <= 1e-13 * std::max(scale, 1e-6));
        if (std::string(fx.name) == "square side") CHECK(clipped > 0);
        std::printf(
            "  (1)(2) %s, w %g: %d moves (%d mixed, %d clipped): swept against the difference of "
            "the sums %.1e, collapse %.1e (band term scale %.1e)\n",
            fx.name,
            fx.w,
            n_moves,
            mixed,
            clipped,
            worst_move,
            worst_collapse,
            scale);
    }
    // (3) the interior flip of a quad (a, b, c, d): (a, b, c) + (a, c, d) -> (a, b, d) + (b, c, d),
    // both faces band, at each fixture's place and three scales: the sums agree to rounding, the
    // swept change of a 2D swap being 0
    {
        double worst = 0., sc = 0.;
        for (const Fixture& fx : fixtures) {
            if (fx.eb == sp) continue;
            for (int trial = 0; trial < 20; ++trial) {
                std::array<Vec2, 4> q;
                for (int k = 0; k < 4; ++k) {
                    const double ang = 0.5 * M_PI * (k + 0.3 * U(rng));
                    q[size_t(k)] =
                        fx.c + fx.w * (0.7 + 0.2 * U(rng)) * Vec2(std::cos(ang), std::sin(ang));
                }
                const double s0 = fx.eb->value({{q[0], q[1], q[2]}}, kDelta) +
                                  fx.eb->value({{q[0], q[2], q[3]}}, kDelta);
                const double s1 = fx.eb->value({{q[0], q[1], q[3]}}, kDelta) +
                                  fx.eb->value({{q[1], q[2], q[3]}}, kDelta);
                worst = std::max(worst, std::abs(s1 - s0) / kDelta);
                sc = std::max(sc, std::abs(s0) / kDelta);
            }
        }
        CHECK(worst <= 1e-13 * sc);
        std::printf("  (3) interior flips: the two band sums %.1e apart (scale %.1e)\n", worst, sc);
    }
    // (4) value() in every corner order
    {
        const Tri t = {{Vec2(1.01, 1.02), Vec2(1.06, 0.97), Vec2(0.99, 1.07)}};
        const double v0 = sq->value(t, kDelta);
        double worst = 0.;
        std::array<int, 3> perm = {{0, 1, 2}};
        do {
            Tri q;
            for (int k = 0; k < 3; ++k) q[size_t(k)] = t[size_t(perm[size_t(k)])];
            worst = std::max(worst, std::abs(sq->value(q, kDelta) - v0));
        } while (std::next_permutation(perm.begin(), perm.end()));
        CHECK(worst <= 1e-15 * std::abs(v0) * 8);
        std::printf("  (4) value() over the 6 corner orders: %.1e of %.3e\n", worst, v0);
    }
}

TEST_CASE("exact-swept-search-model-2d", "[offset][exact_swept][2d]")
{
    // The 2D twin of exact-swept-search-model: BandVolumeEnergy2D::set_swept(), built as the
    // smoother builds it (band_volume_energy_at()), on a square coned to the moving vertex with
    // random labels, at the square's corner, across its side (the offset side clipped, the cut
    // terms' jump) and across the valley's bisector, at two scales:
    //  (1) its value against band_swept_change() from x0, bit for bit, and read back by
    //      swept_value_at() for those inputs only;
    //  (2) its gradient and Hessian against central differences of its value and gradient;
    //  (3) its gradient and Hessian against the "exact" rule's (band_volume_energy_at() under
    //      "exact", the ring's band faces), at x0 and at displaced points: both are E(B)'s, and
    //      no edge part is labelled by rule (a) nor any derivative taken from the ring;
    //  (4) boundary edges crossed by the parabola of a segment and a point (segment_and_point()):
    //      their parts labelled by rule (a), no derivative from the ring; at x0 the edges'
    //      gradient and hess_swept, elsewhere the swept triangles' grad_hess(), and with the value
    //      computed first the same numbers;
    //  (5) there, at x0, the gradient and Hessian against differences of the value (the judged
    //      swept change), where the ring's band faces' gradient (the "exact" rule's, the faces'
    //      rule-(a) bound's) is off: a boundary edge lies on the point's side of the parabola and
    //      its band face mostly on the segment's;
    //  (6) the same at x0 where the members' side lines disagree on a rule-(a) part, decided at
    //      the thin swept triangle's centroid;
    //  (7) boundary edges of B lying on a curve input: at x0 against the "exact" rule's.
    // Errors are relative to max(|reference|, floor), floor 1e-6 of the band term's scale (w for
    // a gradient, 1 for a Hessian): where d is affine on the band's side (the square's side) the
    // Hessian vanishes, and both rules and the differences read rounding there (MEASURED: 1e-16
    // and 3e-13 against 0, 3360x "off" without the floor).
    std::printf("exact-swept-search-model-2d\n");
    std::mt19937 rng(20261112);
    std::uniform_real_distribution<double> U(-1., 1.);
    const auto energy_at = [](TopoOffsetTriMesh& m, const char* rule) {
        m.m_offset_params.band_volume_rule = rule;
        return std::dynamic_pointer_cast<BandVolumeEnergy2D>(m.band_volume_energy_at(0));
    };
    const auto derivs = [](BandVolumeEnergy2D& e, const Vec2& x, Vec2& g, Eigen::Matrix2d& H) {
        const Eigen::VectorXd xv = x;
        Eigen::VectorXd gv;
        Eigen::MatrixXd Hv;
        e.gradient(xv, gv);
        e.hessian(xv, Hv);
        g = gv;
        H = Hv;
    };
    const auto sq = square();
    const auto va = valley();
    std::vector<Fixture> fixtures;
    for (const double w : {0.04, 0.4}) {
        fixtures.push_back({"square corner", sq, Vec2(1.04, 1.03), w});
        fixtures.push_back({"square side", sq, Vec2(1.0, 0.3), w});
        fixtures.push_back({"valley bisector", va, Vec2(0.05, 0.052), w});
    }
    for (const Fixture& fx : fixtures) {
        double worst_g = 0., worst_h = 0., worst_eg = 0., worst_eh = 0.;
        int n_models = 0, n_points = 0, value_mismatch = 0, readback_mismatch = 0;
        size_t evals = 0, rule_a = 0, ring = 0;
        for (int trial = 0; trial < 20; ++trial) {
            std::vector<Vec2> pts;
            std::vector<std::array<int, 3>> F;
            const Vec2 x0 = fx.c + 0.3 * fx.w * Vec2(U(rng), U(rng));
            box(fx.c, fx.w, x0, pts, F);
            std::vector<int> L(F.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            Parameters param;
            auto mesh = make_mesh(param, pts, F, L, fx.eb);
            const auto ex = energy_at(*mesh, "exact");
            const auto sw = energy_at(*mesh, "exact_swept");
            if (!sw) continue; // no boundary edge of B through the vertex
            REQUIRE(ex);
            ++n_models;
            for (int k = 0; k < 3; ++k) {
                const Vec2 x = k == 0 ? x0 : Vec2(x0 + 0.1 * fx.w * Vec2(U(rng), U(rng)));
                const Eigen::VectorXd xv = x;
                ++n_points;
                // (1)
                if (!(sw->value(xv) == mesh->band_swept_change(0, x0, x))) ++value_mismatch;
                {
                    std::vector<BandVolumeEnergy2D::SweptEdge> edges;
                    mesh->band_swept_edges(0, x0, edges);
                    const ExactBandIntegral2D& eb = *mesh->m_band_exact;
                    const double delta = mesh->m_offset_params.target_distance;
                    double E = std::nan("");
                    if (!(sw->swept_value_at(eb, delta, x0, x, edges, E) &&
                          E == mesh->band_swept_change(0, x0, x))) {
                        ++readback_mismatch;
                    }
                    // one ulp off in the position, x0 or an edge's end, or the edge's order: not
                    // its inputs
                    Vec2 y = x;
                    y[0] = std::nextafter(y[0], 2.);
                    auto f2 = edges, f3 = edges;
                    f2.back().a[1] = std::nextafter(f2.back().a[1], 2.);
                    f3.back().x_first = !f3.back().x_first;
                    if (sw->swept_value_at(eb, delta, x0, y, edges, E) ||
                        sw->swept_value_at(eb, delta, y, x, edges, E) ||
                        sw->swept_value_at(eb, delta, x0, x, f2, E) ||
                        sw->swept_value_at(eb, delta, x0, x, f3, E)) {
                        ++readback_mismatch;
                    }
                }
                // (2)
                Vec2 g, ge;
                Eigen::Matrix2d H, He;
                derivs(*sw, x, g, H);
                double eg = std::numeric_limits<double>::infinity(), eh = eg;
                for (const double hr : {2.5e-4, 2.5e-5, 2.5e-6}) {
                    const double h = hr * fx.w;
                    Vec2 gf;
                    Eigen::Matrix2d Hf;
                    for (int j = 0; j < 2; ++j) {
                        Eigen::VectorXd xp = xv, xm = xv;
                        xp[j] += h;
                        xm[j] -= h;
                        gf[j] = (sw->value(xp) - sw->value(xm)) / (2. * h);
                        Vec2 gp, gm;
                        Eigen::Matrix2d Hp, Hm;
                        derivs(*sw, xp, gp, Hp);
                        derivs(*sw, xm, gm, Hm);
                        Hf.col(j) = (gp - gm) / (2. * h);
                    }
                    eg = std::min(eg, (gf - g).norm() / std::max(g.norm(), 1e-6 * fx.w));
                    eh = std::min(eh, (Hf - H).norm() / std::max(H.norm(), 1e-6));
                }
                worst_g = std::max(worst_g, eg);
                worst_h = std::max(worst_h, eh);
                // (3)
                derivs(*ex, x, ge, He);
                worst_eg = std::max(worst_eg, (g - ge).norm() / std::max(ge.norm(), 1e-6 * fx.w));
                worst_eh = std::max(worst_eh, (H - He).norm() / std::max(He.norm(), 1e-6));
            }
            evals += mesh->m_swept_derivs.evals.exchange(0);
            rule_a += mesh->m_swept_derivs.rule_a.exchange(0);
            ring += mesh->m_swept_derivs.ring_cells.exchange(0);
        }
        INFO(fx.name << ", w " << fx.w);
        CHECK(n_models > 0);
        CHECK(value_mismatch == 0);
        CHECK(readback_mismatch == 0);
        CHECK(worst_g <= 1e-6);
        CHECK(worst_h <= 1e-5);
        CHECK(worst_eg <= 1e-9);
        CHECK(worst_eh <= 1e-8);
        CHECK(evals > 0);
        CHECK(rule_a == 0);
        CHECK(ring == 0);
        std::printf(
            "  (1)-(3) %s, w %g: %d models, %d points: value against band_swept_change() %d "
            "mismatches, read back %d; against central differences gradient %.1e, Hessian %.1e; "
            "against \"exact\" gradient %.1e, Hessian %.1e; %zu derivative evaluations, %zu from "
            "the ring\n",
            fx.name,
            fx.w,
            n_models,
            n_points,
            value_mismatch,
            readback_mismatch,
            worst_g,
            worst_h,
            worst_eg,
            worst_eh,
            evals,
            ring);
    }
    // (4) the segment y = 0 and the point (0, 1): their parabola 2 y = x^2 + 1 crosses the square
    // and one of the band's boundary edges through the vertex (no offset side, as there)
    {
        const auto in = segment_and_point();
        const Vec2 c(0.5, 0.6);
        const double w = 0.08;
        std::vector<Vec2> pts;
        std::vector<std::array<int, 3>> F;
        // x0 0.01 above the parabola: the edge to the square's corner (0.58, 0.68) lies on the
        // point's side, its band face (x0, (0.58, 0.6), (0.58, 0.68)) mostly on the segment's
        const Vec2 x0 = c + Vec2(0.01, 0.04);
        box(c, w, x0, pts, F);
        std::vector<int> L(F.size(), 0);
        for (const int i : {0, 1, 2, 3}) L[size_t(i)] = 2; // the faces over the sides y-, x+
        Parameters param;
        auto mesh = make_mesh(param, pts, F, L, in);
        const auto ex = energy_at(*mesh, "exact");
        const auto sw = energy_at(*mesh, "exact_swept");
        REQUIRE(ex);
        REQUIRE(sw);
        std::vector<BandVolumeEnergy2D::SweptEdge> edges;
        mesh->band_swept_edges(0, x0, edges);
        REQUIRE(edges.size() == 2);
        const double delta = mesh->m_offset_params.target_distance;
        const auto pair_at = [](const BandVolumeEnergy2D::SweptEdge& f, const Vec2& y) {
            return f.x_first ? std::array<Vec2, 2>{{y, f.a}} : std::array<Vec2, 2>{{f.a, y}};
        };
        // the edges' grad and (Hs) hess_swept, over delta, rule (a)'s parts counted
        const auto fra = [&](const Vec2& x, Vec2& g, Eigen::Matrix2d& Hs, int& parts) {
            g.setZero();
            Hs.setZero();
            parts = 0;
            for (const auto& f : edges) {
                Vec2 gf;
                Eigen::Matrix2d hf, hs;
                int ra = 0;
                in->front_edge_derivs(pair_at(f, x), delta, f.x_first ? 0 : 1, gf, hf, &ra, &hs);
                g += gf / delta;
                Hs += hs / delta;
                parts += ra;
            }
        };
        // the swept triangles' derivatives in their corner x (swept_change()'s signs), over delta
        const auto swept = [&](const Vec2& x, Vec2& g, Eigen::Matrix2d& H) {
            using wmtk::utils::predicates::orient2d;
            using wmtk::utils::predicates::Orientation;
            g.setZero();
            H.setZero();
            for (const auto& f : edges) {
                const Orientation mv = f.x_first ? orient2d(x0, f.a, x) : orient2d(f.a, x0, x);
                if (mv == Orientation::COLLINEAR) continue;
                const double sign = mv == Orientation::POSITIVE ? -1. : 1.;
                Vec2 gc;
                Eigen::Matrix2d hc;
                in->grad_hess({{x0, x, f.a}}, 1, delta, gc, hc);
                g += sign * gc / delta;
                H += sign * hc / delta;
            }
        };
        double worst_g = 0., worst_h = 0., worst_eg = 0., worst_eh = 0.;
        for (const Vec2& dx : {Vec2(0., 0.), Vec2(0.004, -0.003), Vec2(-0.005, 0.002)}) {
            Vec2 g, ga, ge;
            Eigen::Matrix2d H, Ha, He;
            int pa = 0;
            derivs(*sw, x0 + dx, g, H);
            if (dx.isZero()) {
                fra(x0, ga, Ha, pa);
            } else {
                swept(x0 + dx, ga, Ha);
            }
            derivs(*ex, x0 + dx, ge, He);
            worst_g = std::max(worst_g, (g - ga).norm() / ga.norm());
            worst_h = std::max(worst_h, (H - Ha).norm() / Ha.norm());
            worst_eg = std::max(worst_eg, (g - ge).norm() / ge.norm());
            worst_eh = std::max(worst_eh, (H - He).norm() / He.norm());
        }
        const size_t evals = mesh->m_swept_derivs.evals.exchange(0);
        const size_t rule_a = mesh->m_swept_derivs.rule_a.exchange(0);
        const size_t swept_tris = mesh->m_swept_derivs.swept_tris.exchange(0);
        const size_t swept_x0 = mesh->m_swept_derivs.swept_x0.exchange(0);
        const size_t ring = mesh->m_swept_derivs.ring_cells.exchange(0);
        CHECK(evals == 3);
        CHECK(rule_a == 3);
        CHECK(swept_tris == 2);
        CHECK(swept_x0 == 1);
        CHECK(ring == 0);
        CHECK(worst_g <= 1e-14);
        CHECK(worst_h <= 1e-14);
        {
            // the value first, as the solver's line search: the derivatives are the swept
            // triangles' (the edges skipped), the same numbers
            const Vec2 y = x0 + Vec2(0.003, 0.004);
            const Eigen::VectorXd yv = y;
            sw->value(yv);
            Vec2 g, gs;
            Eigen::Matrix2d H, Hs;
            derivs(*sw, y, g, H);
            swept(y, gs, Hs);
            CHECK(g == gs);
            CHECK(H == Hs);
            CHECK(mesh->m_swept_derivs.swept_tris.exchange(0) == 1);
            CHECK(mesh->m_swept_derivs.rule_a.exchange(0) == 0);
            mesh->m_swept_derivs.evals.exchange(0);
        }
        std::printf(
            "  (4) parabola across the boundary edges: %zu derivative evaluations, %zu with a "
            "rule-(a) part, %zu from the swept triangles, %zu at x0 with their Hessian, %zu from "
            "the ring's band faces; against the edges' grad and hess_swept at x0 and the swept "
            "triangles' elsewhere gradient %.1e, Hessian %.1e; against \"exact\" gradient %.1e, "
            "Hessian %.1e\n",
            evals,
            rule_a,
            swept_tris,
            swept_x0,
            ring,
            worst_g,
            worst_h,
            worst_eg,
            worst_eh);
        // (5)
        Vec2 gA;
        Eigen::Matrix2d HA;
        int parts = 0;
        fra(x0, gA, HA, parts);
        derivs(*sw, x0, gA, HA);
        mesh->m_swept_derivs.evals.exchange(0);
        mesh->m_swept_derivs.rule_a.exchange(0);
        mesh->m_swept_derivs.swept_x0.exchange(0);
        const auto val = [&](const Vec2& dx) {
            const Eigen::VectorXd y = Vec2(x0 + dx);
            return sw->value(y);
        };
        double eg = std::numeric_limits<double>::infinity(), eh = eg;
        double eg_ring = 0.;
        {
            // the ring's band faces' gradient (the "exact" rule's, the bound's): for the record
            Vec2 ge;
            Eigen::Matrix2d He;
            derivs(*ex, x0, ge, He);
            eg_ring = std::numeric_limits<double>::infinity();
            for (const double h : {1e-4, 3e-5, 1e-5}) {
                Vec2 gf;
                for (int i = 0; i < 2; ++i) {
                    const Vec2 ei = h * Vec2::Unit(i);
                    gf[i] = (val(ei) - val(-ei)) / (2. * h);
                }
                eg_ring = std::min(eg_ring, (gf - ge).norm() / gf.norm());
            }
        }
        for (const double h : {1e-4, 3e-5, 1e-5}) {
            Vec2 gf;
            Eigen::Matrix2d Hf;
            for (int i = 0; i < 2; ++i) {
                const Vec2 ei = h * Vec2::Unit(i);
                gf[i] = (val(ei) - val(-ei)) / (2. * h);
                for (int j = 0; j < 2; ++j) {
                    const Vec2 ej = h * Vec2::Unit(j);
                    Hf(i, j) =
                        (val(ei + ej) - val(ei - ej) - val(ej - ei) + val(-ei - ej)) / (4. * h * h);
                }
            }
            eg = std::min(eg, (gf - gA).norm() / gA.norm());
            eh = std::min(eh, (Hf - HA).norm() / HA.norm());
        }
        CHECK(parts > 0);
        CHECK(eg <= 1e-6);
        CHECK(eh <= 1e-5);
        CHECK(eg_ring > 1e-4); // the premise: the faces' rule (a) is not the edges'
        std::printf(
            "  (5) the model across the same edges: %d rule-(a) parts; at x0 against differences "
            "of its value gradient %.1e, Hessian %.1e (the ring's band faces' gradient %.1e)\n",
            parts,
            eg,
            eh,
            eg_ring);
    }
    // (6) rule (a) where the members' side lines disagree: a segment on y = 0, its offset side
    // y > 0, and the point (0, -1) on its far side; their parabola 2 y = -(x^2 + 1). Below it the
    // band's parts take the point, above it they are dropped, decided by the member nearest at
    // the part's centroid. The boundary edge from x0 up to (0.5, -0.5) crosses the parabola at
    // 0.42 of its length: the thin swept triangle's centroid, a third of the way from x0, is on
    // the point's side and keeps the edge's part; its midpoint, 3D's label_face() choice, is on
    // the segment's and would drop it. At x0 the model's derivatives against differences of its
    // value.
    {
        Eigen::MatrixXd V(3, 2);
        V << 5, 0, -5, 0, 0, -1;
        Eigen::MatrixXi E(1, 2);
        E << 0, 1;
        const auto in = std::make_shared<ExactBandIntegral2D>(
            V,
            E,
            std::vector<int>{2},
            std::vector<int8_t>{1},
            1);
        const Vec2 c(0.5, -0.62);
        const double w = 0.12;
        const Vec2 x0(0.5, -0.7155);
        std::vector<Vec2> pts;
        std::vector<std::array<int, 3>> F;
        box(c, w, x0, pts, F);
        std::vector<int> L(F.size(), 0);
        for (const int i : {5, 6, 7}) L[size_t(i)] = 2; // (x0, p6, p7), (x0, p7, p8), (x0, p8, p1)
        Parameters param;
        auto mesh = make_mesh(param, pts, F, L, in);
        const auto sw = energy_at(*mesh, "exact_swept");
        REQUIRE(sw);
        std::vector<BandVolumeEnergy2D::SweptEdge> edges;
        mesh->band_swept_edges(0, x0, edges);
        REQUIRE(edges.size() == 2);
        // the premises: on the edge to p6 = (0.5, -0.5) the point is nearer at a third of the way
        // from x0 and the segment at its midpoint
        const Vec2 p6 = pts[6];
        const auto point_nearer = [](const Vec2& y) {
            return (y - Vec2(0., -1.)).norm() < std::abs(y.y());
        };
        REQUIRE(point_nearer(x0 + (p6 - x0) / 3.));
        REQUIRE(!point_nearer(0.5 * (x0 + p6)));
        Vec2 g_edge = Vec2::Zero();
        for (const auto& f : edges) {
            if (!(f.a == p6)) continue;
            Eigen::Matrix2d hf;
            const std::array<Vec2, 2> pr =
                f.x_first ? std::array<Vec2, 2>{{x0, f.a}} : std::array<Vec2, 2>{{f.a, x0}};
            in->front_edge_derivs(pr, kDelta, f.x_first ? 0 : 1, g_edge, hf);
            g_edge /= kDelta;
        }
        Vec2 g;
        Eigen::Matrix2d H;
        derivs(*sw, x0, g, H);
        const auto val = [&](const Vec2& dx) {
            const Eigen::VectorXd y = Vec2(x0 + dx);
            return sw->value(y);
        };
        double eg = std::numeric_limits<double>::infinity(), eh = eg, drop = eg;
        for (const double h : {1e-4, 3e-5, 1e-5}) {
            Vec2 gf;
            Eigen::Matrix2d Hf;
            for (int i = 0; i < 2; ++i) {
                const Vec2 ei = h * Vec2::Unit(i);
                gf[i] = (val(ei) - val(-ei)) / (2. * h);
                for (int j = 0; j < 2; ++j) {
                    const Vec2 ej = h * Vec2::Unit(j);
                    Hf(i, j) =
                        (val(ei + ej) - val(ei - ej) - val(ej - ei) + val(-ei - ej)) / (4. * h * h);
                }
            }
            eg = std::min(eg, (gf - g).norm() / gf.norm());
            eh = std::min(eh, (Hf - H).norm() / Hf.norm());
            drop = std::min(drop, (gf - (g - g_edge)).norm() / gf.norm());
        }
        CHECK(mesh->m_swept_derivs.rule_a.exchange(0) == 1);
        CHECK(mesh->m_swept_derivs.ring_cells.exchange(0) == 0);
        CHECK(g_edge.norm() > 0.);
        CHECK(eg <= 1e-6);
        CHECK(eh <= 1e-5);
        CHECK(drop > 1e-2); // the edge's part is what the differences see
        std::printf(
            "  (6) side lines disagreeing: at x0 against differences of the value gradient %.1e, "
            "Hessian %.1e; with the crossed edge's part dropped (its midpoint's choice) gradient "
            "%.1e\n",
            eg,
            eh,
            drop);
    }
    // (7) boundary edges of B lying on the input: a lone segment on y = 0 (a curve, no side),
    // the band below it, the input faces above, x0 on it. The edges' parts lie on the segment's
    // line, where d has its kink: grad d from the band's side, as the ring's band faces take it
    // (the one-sided limit into each face). At x0 the model's derivatives against the "exact"
    // rule's.
    {
        Eigen::MatrixXd V(2, 2);
        V << -5, 0, 5, 0;
        Eigen::MatrixXi E(1, 2);
        E << 0, 1;
        const auto in = std::make_shared<ExactBandIntegral2D>(V, E, std::vector<int>{});
        const Vec2 c(0.3, 0.);
        const double w = 0.04;
        const Vec2 x0(0.305, 0.);
        std::vector<Vec2> pts;
        std::vector<std::array<int, 3>> F;
        box(c, w, x0, pts, F);
        std::vector<int> L(F.size(), 1);
        for (const int i : {0, 1, 2, 7}) L[size_t(i)] = 2; // the faces below y = 0
        Parameters param;
        auto mesh = make_mesh(param, pts, F, L, in);
        const auto ex = energy_at(*mesh, "exact");
        const auto sw = energy_at(*mesh, "exact_swept");
        REQUIRE(ex);
        REQUIRE(sw);
        std::vector<BandVolumeEnergy2D::SweptEdge> edges;
        mesh->band_swept_edges(0, x0, edges);
        REQUIRE(edges.size() == 2);
        for (const auto& f : edges) REQUIRE(f.a.y() == 0.);
        Vec2 g, ge;
        Eigen::Matrix2d H, He;
        derivs(*sw, x0, g, H);
        derivs(*ex, x0, ge, He);
        const double eg = (g - ge).norm() / ge.norm(), eh = (H - He).norm() / He.norm();
        CHECK(mesh->m_swept_derivs.rule_a.exchange(0) == 0);
        CHECK(mesh->m_swept_derivs.ring_cells.exchange(0) == 0);
        CHECK(eg <= 1e-9);
        CHECK(eh <= 1e-8);
        std::printf(
            "  (7) boundary edges on a curve input, the band below: at x0 against \"exact\" "
            "gradient %.1e, Hessian %.1e\n",
            eg,
            eh);
    }
}

TEST_CASE("exact-swept-accept-2d", "[offset][exact_swept][2d]")
{
    // The rules that read the swept change, on the square coned to a vertex at the square's
    // corner, across its side and across the valley's bisector, random labels:
    //  (1) swept_accept(): a move that raises the AMIPS terms plus the swept change at full step
    //      and lowers it at a halving stands at the first such halving, counted there; with no
    //      halving allowed (the projected step's) it is refused; and its decision is the "exact"
    //      rule's, the ring's sum of tri_energy(), at every position tested (the swept change is
    //      exact here);
    //  (2) the collapse rule (collapse_before_vertex(), collapse_edge_after()) through the engine's
    //      collapse_edge(): under "exact_swept" each collapse of the vertex onto a ring vertex is
    //      decided as under "exact", the same result mesh; the AMIPS terms alone (the swept term
    //      left out) decide otherwise in some.
    std::printf("exact-swept-accept-2d\n");
    std::mt19937 rng(20261113);
    std::uniform_real_distribution<double> U(-1., 1.);
    const auto sq = square();
    const auto va = valley();
    const std::vector<Fixture> fixtures = {
        {"square corner", sq, Vec2(1.04, 1.03), 0.04},
        {"square side", sq, Vec2(1.0, 0.3), 0.04},
        {"valley bisector", va, Vec2(0.05, 0.052), 0.04}};
    // An octagon around c (no three boundary points collinear, so every collapse of the vertex
    // onto a boundary point leaves valid faces), coned to m.
    const auto octagon = [&](const Fixture& fx,
                             const Vec2& m,
                             std::vector<Vec2>& pts,
                             std::vector<std::array<int, 3>>& F) {
        pts.assign(1, m);
        for (int k = 0; k < 8; ++k) {
            const double a = M_PI / 4. * (k + 0.2 * U(rng));
            pts.push_back(fx.c + fx.w * Vec2(std::cos(a), std::sin(a)));
        }
        F.clear();
        for (int i = 1; i <= 8; ++i) F.push_back({{0, i, i % 8 + 1}});
    };
    for (const Fixture& fx : fixtures) {
        // (1)
        int found = 0, at_halving = 0, refused_without = 0, disagree = 0;
        for (int trial = 0; trial < 200 && found < 8; ++trial) {
            std::vector<Vec2> pts;
            std::vector<std::array<int, 3>> F;
            const Vec2 x0 = fx.c + 0.3 * fx.w * Vec2(U(rng), U(rng));
            octagon(fx, x0, pts, F);
            std::vector<int> L(F.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            Parameters param;
            param.offset_amips_weight = 1e-3; // the band term matters against the AMIPS term
            auto mesh = make_mesh(param, pts, F, L, fx.eb);
            const std::vector<size_t> ring = mesh->get_one_ring_fids_for_vertex(0);
            const double amips0 = mesh->tri_energy_sum(ring, false);
            const double e0 = mesh->tri_energy_sum(ring);
            const Vec2 x1 = fx.c + 0.7 * fx.w * Vec2(U(rng), U(rng));
            // E's change (the "exact" rule's) at x0 + 2^-k (x1 - x0), k = 0..4
            std::array<double, 5> dE;
            bool valid = true;
            for (int k = 0; k <= 4; ++k) {
                mesh->set_vertex_position(0, x0 + std::ldexp(1., -k) * (x1 - x0));
                for (const size_t f : ring) valid = valid && !mesh->is_inverted(f);
                dE[size_t(k)] = mesh->tri_energy_sum(ring) - e0;
            }
            mesh->set_vertex_position(0, x0);
            if (!valid || !(dE[0] > 0.)) continue;
            int first = -1;
            for (int k = 1; k <= 4 && first < 0; ++k) {
                if (dE[size_t(k)] <= 0.) first = k;
            }
            if (first < 0) continue;
            ++found;
            TopoOffsetTriMesh::SweptCounters c;
            mesh->set_vertex_position(0, x1);
            const bool ok = mesh->swept_accept(0, ring, x0, amips0, c);
            const Vec2 kept = mesh->m_vertex_attribute[0].m_posf;
            if (ok && c.accepted[size_t(first)] == 1 &&
                kept == Vec2(x0 + std::ldexp(1., -first) * (x1 - x0))) {
                ++at_halving;
            } else {
                ++disagree;
            }
            TopoOffsetTriMesh::SweptCounters c0;
            mesh->set_vertex_position(0, x1);
            if (!mesh->swept_accept(0, ring, x0, amips0, c0, 0) && c0.refused == 1) {
                ++refused_without;
            }
        }
        INFO(fx.name);
        CHECK(found > 0);
        CHECK(at_halving == found);
        CHECK(refused_without == found);
        CHECK(disagree == 0);
        std::printf(
            "  (1) %s: %d moves rising at full step and falling at a halving: %d kept at the "
            "first such halving, %d refused without halving\n",
            fx.name,
            found,
            at_halving,
            refused_without);
        // (2)
        int n_coll = 0, same = 0, accepted = 0, amips_only_differs = 0;
        for (int trial = 0; trial < 40; ++trial) {
            std::vector<Vec2> pts;
            std::vector<std::array<int, 3>> F;
            const Vec2 x0 = fx.c + 0.3 * fx.w * Vec2(U(rng), U(rng));
            octagon(fx, x0, pts, F);
            std::vector<int> L(F.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            const size_t v2 = size_t(1 + trial % 8);
            std::array<bool, 2> ok;
            std::array<std::vector<std::array<double, 6>>, 2> result;
            double amips_change = 0.;
            for (const int r : {0, 1}) {
                Parameters param;
                param.offset_amips_weight = 1e-3;
                auto mesh = make_mesh(param, pts, F, L, fx.eb, r == 0 ? "exact" : "exact_swept");
                if (r == 1) {
                    // the AMIPS terms alone: v1's ring before, the faces it keeps renamed after
                    amips_change =
                        -mesh->tri_energy_sum(mesh->get_one_ring_fids_for_vertex(0), false);
                    for (const size_t f : mesh->get_one_ring_fids_for_vertex(0)) {
                        auto vs = mesh->oriented_tri_vids(f);
                        if (std::find(vs.begin(), vs.end(), v2) != vs.end()) continue;
                        for (size_t& u : vs) {
                            if (u == 0) u = v2;
                        }
                        amips_change += mesh->cell_energy(vs, mesh->get_quality(vs), false);
                    }
                }
                TopoOffsetTriMesh::Tuple t =
                    std::get<0>(mesh->tuple_from_edge(std::array<size_t, 2>{{size_t(0), v2}}));
                if (t.vid(*mesh) != 0) t = t.switch_vertex(*mesh);
                std::vector<TopoOffsetTriMesh::Tuple> nt;
                ok[size_t(r)] = mesh->collapse_edge(t, nt);
                for (const auto& f : mesh->get_faces()) {
                    const auto vs = mesh->oriented_tri_vids(f.fid(*mesh));
                    std::array<double, 6> row;
                    for (int k = 0; k < 3; ++k) {
                        row[size_t(2 * k)] = mesh->m_vertex_attribute[vs[size_t(k)]].m_posf[0];
                        row[size_t(2 * k + 1)] = mesh->m_vertex_attribute[vs[size_t(k)]].m_posf[1];
                    }
                    result[size_t(r)].push_back(row);
                }
                std::sort(result[size_t(r)].begin(), result[size_t(r)].end());
            }
            ++n_coll;
            if (ok[0] == ok[1] && result[0] == result[1]) ++same;
            if (ok[1]) ++accepted;
            if ((amips_change <= 0.) != ok[1]) ++amips_only_differs;
        }
        CHECK(same == n_coll);
        CHECK(accepted > 0);
        CHECK(accepted < n_coll);
        CHECK(amips_only_differs > 0);
        std::printf(
            "  (2) %s: %d collapses, %d decided and made as under \"exact\", %d accepted; the "
            "AMIPS terms alone decide %d otherwise\n",
            fx.name,
            n_coll,
            same,
            accepted,
            amips_only_differs);
    }
}
