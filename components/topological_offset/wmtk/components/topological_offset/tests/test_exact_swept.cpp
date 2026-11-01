#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/components/topological_offset/ExactBandIntegral.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <Eigen/Geometry>
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
 * EXPERIMENTAL_band_volume_rule "exact_swept" (TopoOffsetTetMesh, EXACT SWEPT CHANGE):
 * band_swept_change() and band_chain_change() against the change of the sum of value() over the
 * band cells, and the search model (BandVolumeEnergy3D::set_swept()) against band_swept_change(),
 * central differences and the "exact" rule's derivatives.
 */

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;
constexpr double kDelta = 0.1;

/// The cube [-1, 1]^3, every row's normal out of the solid.
void cube(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(8, 3);
    V << -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1;
    F.resize(12, 3);
    F << 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6,
        3, 0, 4, 3, 4, 7;
}

/// The valley: A in z = 0 (y >= 0), B in y = 0 (z >= 0, row reversed so both normals point into
/// y, z > 0, the offset side); there d = min(y, z).
void valley(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(4, 3);
    V << -10, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10;
    F.resize(2, 3);
    F << 0, 1, 2, 1, 0, 3;
}

/// The input's primitives as the mesh's band term reads them (m_input_complex_envelope), and the
/// exact integral over them.
struct Input
{
    std::shared_ptr<Envelope> env;
    double pad = 0.;
    std::shared_ptr<ExactBandIntegral> eb;
};
Input input(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    Input in;
    in.env = std::make_shared<Envelope>(true);
    in.env->init(V, F, 0.);
    in.pad = input_pad(V);
    in.eb = std::make_shared<ExactBandIntegral>(
        V,
        F,
        std::vector<int8_t>(size_t(F.rows()), 1),
        1,
        size_t(1024),
        in.env);
    return in;
}

/// A mesh of the cells T on the points V, every cell labelled L[i], every face untracked, the
/// band term read from `in`.
std::unique_ptr<TopoOffsetTetMesh> make_mesh(
    Parameters& param,
    const std::vector<Vec3>& pts,
    const std::vector<std::array<int, 4>>& T,
    const std::vector<int>& L,
    const Input& in)
{
    param.target_distance = kDelta;
    param.front_conv = 1e-3;
    param.stencil_order = 1;
    param.band_volume_rule = "exact_swept";
    Eigen::MatrixXd V(int(pts.size()), 3);
    for (size_t i = 0; i < pts.size(); ++i) V.row(int(i)) = pts[i].transpose();
    Eigen::MatrixXi Tm(int(T.size()), 4);
    for (size_t i = 0; i < T.size(); ++i) {
        std::array<int, 4> t = T[i];
        const auto p = [&](int k) { return pts[size_t(t[size_t(k)])]; };
        if ((p(1) - p(0)).dot((p(2) - p(0)).cross(p(3) - p(0))) < 0.) std::swap(t[2], t[3]);
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = t[size_t(j)];
    }
    auto mesh = std::make_unique<TopoOffsetTetMesh>(param, 0);
    MatrixSi Tags(int(T.size()), 1);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh->init_from_image(V, Tm, Tags, V_env, F_env, {"a"});
    for (const auto& f : mesh->get_faces()) {
        mesh->m_face_attribute[f.fid(*mesh)].m_is_surface_fs = false;
    }
    for (size_t v = 0; v < pts.size(); ++v) mesh->m_vertex_attribute[v].m_is_on_surface = false;
    for (size_t i = 0; i < T.size(); ++i) {
        REQUIRE(!mesh->is_inverted(mesh->tuple_from_tet(i)));
        mesh->m_tet_attribute[i].label = L[i];
    }
    mesh->m_input_complex_envelope = in.env;
    mesh->m_input_complex_pad = in.pad;
    mesh->m_band_exact = in.eb;
    return mesh;
}

/// A box of half-width w around c, its 6 sides split in 2 triangles each and coned to m: the
/// points (m, then the 8 corners) and 12 cells.
void box(
    const Vec3& c,
    const double w,
    const Vec3& m,
    std::vector<Vec3>& pts,
    std::vector<std::array<int, 4>>& T)
{
    pts.assign(1, m);
    for (int k = 0; k < 8; ++k) {
        pts.push_back(c + w * Vec3(k & 1 ? 1. : -1., k & 2 ? 1. : -1., k & 4 ? 1. : -1.));
    }
    const int sides[6][4] =
        {{0, 1, 3, 2}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 3, 7, 5}};
    T.clear();
    for (const auto& s : sides) {
        T.push_back({{0, 1 + s[0], 1 + s[1], 1 + s[2]}});
        T.push_back({{0, 1 + s[0], 1 + s[2], 1 + s[3]}});
    }
}

} // namespace

TEST_CASE("exact-swept-identities", "[offset][exact_swept]")
{
    // EXPERIMENTAL_band_volume_rule "exact_swept" (TopoOffsetTetMesh, EXACT SWEPT CHANGE):
    //  (1) a vertex move: band_swept_change() against the change of the sum of value() over the
    //      band cells, on a box coned to the moving vertex with random labels (band, background,
    //      input), random moves inside the box -- mixed signs over the boundary faces -- at the
    //      cube's corner and across the valley's bisector;
    //  (2) a collapse: the same with the vertex moved onto a box corner, the cells through it
    //      flat;
    //  (3) a swap: band_chain_change() against the sums over a 3-2 and a 4-4 configuration,
    //      random band subsets of the old and the new cells;
    //  (4) value() does not depend on the corner order.
    std::printf("exact-swept-identities\n");
    Eigen::MatrixXd Vc, Vv;
    Eigen::MatrixXi Fc, Fv;
    cube(Vc, Fc);
    valley(Vv, Fv);
    const Input in_cube = input(Vc, Fc), in_valley = input(Vv, Fv);
    std::mt19937 rng(20261022);
    std::uniform_real_distribution<double> U(-1., 1.);
    const auto cell_pts = [](TopoOffsetTetMesh& m, const std::array<size_t, 4>& t) {
        Tet p;
        for (int k = 0; k < 4; ++k) p[size_t(k)] = m.m_vertex_attribute[t[size_t(k)]].m_posf;
        return p;
    };
    const auto band_sum = [&](TopoOffsetTetMesh& m, const Input& in, const size_t n) {
        double s = 0.;
        for (size_t tid = 0; tid < n; ++tid) {
            if (m.m_tet_attribute[tid].label != 2) continue;
            s += in.eb->value(cell_pts(m, m.oriented_tet_vids(tid)), kDelta);
        }
        return s / kDelta;
    };
    for (const auto& [name, in, c] :
         {std::tuple{"cube corner", &in_cube, Vec3(1.04, 1.03, 1.05)},
          std::tuple{"valley bisector", &in_valley, Vec3(0.2, 0.05, 0.052)}}) {
        double worst_move = 0., worst_collapse = 0., scale = 0.;
        int mixed = 0, n_moves = 0;
        for (int trial = 0; trial < 40; ++trial) {
            std::vector<Vec3> pts;
            std::vector<std::array<int, 4>> T;
            const double w = 0.04;
            const Vec3 m0 = c + 0.3 * w * Vec3(U(rng), U(rng), U(rng));
            box(c, w, m0, pts, T);
            std::vector<int> L(T.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            Parameters param;
            auto mesh = make_mesh(param, pts, T, L, *in);
            mesh->m_offset_params.band_volume_rule = "exact_swept";
            const size_t n = T.size();
            const double before = band_sum(*mesh, *in, n);
            // (1) a move inside the box
            const Vec3 x0 = m0, x1 = c + 0.6 * w * Vec3(U(rng), U(rng), U(rng));
            const double swept = mesh->band_swept_change(0, x0, x1);
            mesh->m_vertex_attribute[0].m_posf = x1;
            const double after = band_sum(*mesh, *in, n);
            mesh->m_vertex_attribute[0].m_posf = x0;
            scale = std::max(scale, std::abs(before));
            worst_move = std::max(worst_move, std::abs(swept - (after - before)));
            ++n_moves;
            {
                // mixed: boundary faces of B through the vertex swept both ways
                int pos = 0, neg = 0;
                for (size_t tid = 0; tid < n; ++tid) {
                    if (L[tid] != 2) continue;
                    const auto v = mesh->oriented_tet_vids(tid);
                    for (int i = 0; i < 4; ++i) {
                        if (v[size_t(i)] == 0) continue;
                        std::array<size_t, 3> f = {
                            {v[size_t((i + 1) % 4)],
                             v[size_t((i + 2) % 4)],
                             v[size_t((i + 3) % 4)]}};
                        const auto [ft, fid] = mesh->tuple_from_face(f);
                        (void)fid;
                        const auto o = ft.switch_tetrahedron(*mesh);
                        const size_t other =
                            ft.tid(*mesh) == tid ? (o ? o->tid(*mesh) : size_t(-1)) : ft.tid(*mesh);
                        if (other != size_t(-1) && L[other] == 2) continue;
                        const Vec3 P = mesh->m_vertex_attribute[v[size_t(i)]].m_posf;
                        std::vector<Vec3> q;
                        for (const size_t u : f) {
                            if (u != 0) q.push_back(mesh->m_vertex_attribute[u].m_posf);
                        }
                        Vec3 N = (q[0] - x0).cross(q[1] - x0);
                        if (N.dot(P - x0) > 0.) N = -N;
                        (N.dot(x1 - x0) > 0. ? pos : neg) += 1;
                    }
                }
                if (pos > 0 && neg > 0) ++mixed;
            }
            // (2) a collapse of the vertex onto the box corner 1 (point 1)
            {
                const Vec3 x2 = mesh->m_vertex_attribute[1].m_posf;
                const double sw = mesh->band_swept_change(0, x0, x2);
                double old_sum = 0., new_sum = 0.;
                for (size_t tid = 0; tid < n; ++tid) {
                    if (L[tid] != 2) continue;
                    const auto v = mesh->oriented_tet_vids(tid);
                    old_sum += in->eb->value(cell_pts(*mesh, v), kDelta);
                    if (std::find(v.begin(), v.end(), size_t(1)) != v.end()) continue; // flat
                    Tet p = cell_pts(*mesh, v);
                    for (int k = 0; k < 4; ++k) {
                        if (v[size_t(k)] == 0) p[size_t(k)] = x2;
                    }
                    new_sum += in->eb->value(p, kDelta);
                }
                worst_collapse =
                    std::max(worst_collapse, std::abs(sw - (new_sum - old_sum) / kDelta));
            }
        }
        // in units of the band term, against the box's band term scale
        CHECK(worst_move <= 1e-13 * std::max(scale, 1e-6));
        CHECK(worst_collapse <= 1e-13 * std::max(scale, 1e-6));
        CHECK(mixed > 0);
        std::printf(
            "  (1)(2) %s: %d moves (%d with boundary faces swept both ways): swept against the "
            "difference of the sums %.1e, collapse %.1e (band term scale %.1e)\n",
            name,
            n_moves,
            mixed,
            worst_move,
            worst_collapse,
            scale);
    }
    // (3) swaps: 3-2 around edge (a, b) with ring c, d, e; 4-4 around (a, b) with ring c0..c3
    {
        const Vec3 c = Vec3(1.03, 1.02, 0.4);
        std::vector<Vec3> pts = {
            c + Vec3(0., 0., -0.03), // a
            c + Vec3(0.002, -0.001, 0.03), // b
            c + Vec3(0.03, 0., 0.), // c / c0
            c + Vec3(-0.015, 0.026, 0.001), // d / c1
            c + Vec3(-0.015, -0.026, -0.001), // e / c2
            c + Vec3(0.01, -0.03, 0.002)}; // c3
        // the mesh: the 4-4's old cells, so that every point is a vertex of it
        std::vector<std::array<int, 4>> T = {
            {{0, 1, 2, 3}},
            {{0, 1, 3, 4}},
            {{0, 1, 4, 5}},
            {{0, 1, 5, 2}}};
        Parameters param;
        auto mesh = make_mesh(param, pts, T, {2, 2, 2, 2}, in_cube);
        mesh->m_offset_params.band_volume_rule = "exact_swept";
        const auto sum = [&](const std::vector<std::array<size_t, 4>>& cells) {
            double s = 0.;
            for (const auto& t : cells) s += in_cube.eb->value(cell_pts(*mesh, t), kDelta);
            return s / kDelta;
        };
        const std::vector<std::array<size_t, 4>> old32 = {
            {{0, 1, 2, 3}},
            {{0, 1, 3, 4}},
            {{0, 1, 4, 2}}};
        const std::vector<std::array<size_t, 4>> new32 = {{{0, 2, 3, 4}}, {{1, 2, 4, 3}}};
        double worst = 0.;
        int calls_flip = -1;
        for (int trial = 0; trial < 64; ++trial) {
            std::vector<std::array<size_t, 4>> ob, nb;
            for (size_t i = 0; i < old32.size(); ++i) {
                if (trial & (1 << i)) ob.push_back(old32[i]);
            }
            for (size_t i = 0; i < new32.size(); ++i) {
                if (trial & (8 << i)) nb.push_back(new32[i]);
            }
            const double ch = mesh->band_chain_change(ob, nb);
            worst = std::max(worst, std::abs(ch - (sum(nb) - sum(ob))));
        }
        // the surface flip: (a, b, c, d) band, the rest not, the new cells not: one call
        {
            ExactBandIntegral::Stats st;
            const double ch = mesh->band_chain_change({old32[0]}, {}, &st);
            calls_flip = int(st.cells);
            worst = std::max(worst, std::abs(ch + sum({old32[0]})));
        }
        // 4-4 around (a, b): ring c0 c1 c2 c3 (points 2, 3, 4, 5), new diagonal (c0, c2)
        const std::vector<size_t> r = {2, 3, 4, 5}; // by angle about (a, b)
        std::vector<std::array<size_t, 4>> o44, n44;
        for (int i = 0; i < 4; ++i) o44.push_back({{0, 1, r[size_t(i)], r[size_t((i + 1) % 4)]}});
        n44 = {
            {{0, r[0], r[1], r[2]}},
            {{1, r[0], r[1], r[2]}},
            {{0, r[0], r[2], r[3]}},
            {{1, r[0], r[2], r[3]}}};
        for (int trial = 0; trial < 256; ++trial) {
            std::vector<std::array<size_t, 4>> ob, nb;
            for (int i = 0; i < 4; ++i) {
                if (trial & (1 << i)) ob.push_back(o44[size_t(i)]);
                if (trial & (16 << i)) nb.push_back(n44[size_t(i)]);
            }
            const double ch = mesh->band_chain_change(ob, nb);
            worst = std::max(worst, std::abs(ch - (sum(nb) - sum(ob))));
        }
        const double scale = std::abs(sum(o44));
        CHECK(worst <= 1e-13 * scale);
        CHECK(calls_flip == 1);
        CHECK(mesh->band_chain_change(o44, n44) == Catch::Approx(0.).margin(1e-14 * scale));
        std::printf(
            "  (3) 3-2 and 4-4 swaps, random band subsets: chain change against the sums %.1e "
            "(scale %.1e); a surface 3-2 flip: %d value() call\n",
            worst,
            scale,
            calls_flip);
    }
    // (4) value() in every corner order
    {
        const Tet t = {
            {Vec3(1.01, 1.02, 0.97),
             Vec3(1.06, 1.0, 1.03),
             Vec3(0.99, 1.07, 1.02),
             Vec3(1.03, 1.04, 1.08)}};
        const double v0 = in_cube.eb->value(t, kDelta);
        double worst = 0.;
        std::array<int, 4> perm = {{0, 1, 2, 3}};
        do {
            Tet q;
            for (int k = 0; k < 4; ++k) q[size_t(k)] = t[size_t(perm[size_t(k)])];
            worst = std::max(worst, std::abs(in_cube.eb->value(q, kDelta) - v0));
        } while (std::next_permutation(perm.begin(), perm.end()));
        CHECK(worst <= 1e-15 * std::abs(v0) * 8);
        std::printf("  (4) value() over the 24 corner orders: %.1e of %.3e\n", worst, v0);
    }
}

TEST_CASE("exact-swept-search-model", "[offset][exact_swept]")
{
    // BandVolumeEnergy3D::set_swept(), built as the smoother builds it (band_volume_energy_at()),
    // on a box coned to the moving vertex with random labels (exact-swept-identities), at the
    // cube's corner and across the valley's bisector:
    //  (1) its value against band_swept_change() from x0, bit for bit;
    //  (2) its gradient and Hessian against central differences of its value and gradient;
    //  (3) its gradient and Hessian against the "exact" rule's (band_volume_energy_at() under
    //      "exact"), at x0 and at displaced points: both are E(B)'s, and no derivative is taken
    //      from the ring's band cells;
    //  (4) front faces crossed by the curved bisector of a face and an edge (the input of
    //      exact-band-front-curved): the derivatives are the "exact" rule's, counted as taken
    //      from the ring's band cells.
    std::printf("exact-swept-search-model\n");
    Eigen::MatrixXd Vc, Vv;
    Eigen::MatrixXi Fc, Fv;
    cube(Vc, Fc);
    valley(Vv, Fv);
    const Input in_cube = input(Vc, Fc), in_valley = input(Vv, Fv);
    std::mt19937 rng(20261023);
    std::uniform_real_distribution<double> U(-1., 1.);
    // vertex 0's band term under `rule`, as the smoother builds it
    const auto energy_at = [](TopoOffsetTetMesh& m, const char* rule) {
        m.m_offset_params.band_volume_rule = rule;
        return std::dynamic_pointer_cast<BandVolumeEnergy3D>(m.band_volume_energy_at(0));
    };
    const auto derivs = [](BandVolumeEnergy3D& e, const Vec3& x, Vec3& g, Eigen::Matrix3d& H) {
        const Eigen::VectorXd xv = x;
        Eigen::VectorXd gv;
        Eigen::MatrixXd Hv;
        e.gradient(xv, gv);
        e.hessian(xv, Hv);
        g = gv;
        H = Hv;
    };
    for (const auto& [name, in, c] :
         {std::tuple{"cube corner", &in_cube, Vec3(1.04, 1.03, 1.05)},
          std::tuple{"valley bisector", &in_valley, Vec3(0.2, 0.05, 0.052)}}) {
        double worst_g = 0., worst_h = 0., worst_eg = 0., worst_eh = 0.;
        int n_models = 0, n_points = 0, value_mismatch = 0;
        size_t evals = 0, ring = 0;
        for (int trial = 0; trial < 20; ++trial) {
            std::vector<Vec3> pts;
            std::vector<std::array<int, 4>> T;
            const double w = 0.04;
            const Vec3 x0 = c + 0.3 * w * Vec3(U(rng), U(rng), U(rng));
            box(c, w, x0, pts, T);
            std::vector<int> L(T.size());
            for (int& l : L)
                l = std::uniform_int_distribution<int>(0, 5)(rng) < 3 ? 2 : (trial % 2);
            Parameters param;
            auto mesh = make_mesh(param, pts, T, L, *in);
            const auto ex = energy_at(*mesh, "exact");
            const auto sw = energy_at(*mesh, "exact_swept");
            if (!sw) continue; // no boundary face of B through the vertex
            REQUIRE(ex);
            ++n_models;
            for (int k = 0; k < 3; ++k) {
                const Vec3 x = k == 0 ? x0 : Vec3(x0 + 0.1 * w * Vec3(U(rng), U(rng), U(rng)));
                const Eigen::VectorXd xv = x;
                ++n_points;
                // (1)
                if (!(sw->value(xv) == mesh->band_swept_change(0, x0, x))) ++value_mismatch;
                // (2)
                Vec3 g, ge;
                Eigen::Matrix3d H, He;
                derivs(*sw, x, g, H);
                double eg = std::numeric_limits<double>::infinity(), eh = eg;
                for (const double h : {1e-5, 1e-6, 1e-7}) {
                    Vec3 gf;
                    Eigen::Matrix3d Hf;
                    for (int j = 0; j < 3; ++j) {
                        Eigen::VectorXd xp = xv, xm = xv;
                        xp[j] += h;
                        xm[j] -= h;
                        gf[j] = (sw->value(xp) - sw->value(xm)) / (2. * h);
                        Vec3 gp, gm;
                        Eigen::Matrix3d Hp, Hm;
                        derivs(*sw, xp, gp, Hp);
                        derivs(*sw, xm, gm, Hm);
                        Hf.col(j) = (gp - gm) / (2. * h);
                    }
                    eg = std::min(eg, (gf - g).norm() / g.norm());
                    eh = std::min(eh, (Hf - H).norm() / H.norm());
                }
                worst_g = std::max(worst_g, eg);
                worst_h = std::max(worst_h, eh);
                // (3)
                derivs(*ex, x, ge, He);
                worst_eg = std::max(worst_eg, (g - ge).norm() / ge.norm());
                worst_eh = std::max(worst_eh, (H - He).norm() / He.norm());
            }
            evals += mesh->m_swept_derivs.evals.exchange(0);
            ring += mesh->m_swept_derivs.ring_cells.exchange(0);
        }
        CHECK(n_models > 0);
        CHECK(value_mismatch == 0);
        CHECK(worst_g <= 1e-6);
        CHECK(worst_h <= 1e-5);
        CHECK(worst_eg <= 1e-9);
        CHECK(worst_eh <= 1e-8);
        CHECK(evals > 0);
        CHECK(ring == 0);
        std::printf(
            "  (1)-(3) %s: %d models, %d points: value against band_swept_change() %d "
            "mismatches; against central differences gradient %.1e, Hessian %.1e; against "
            "\"exact\" gradient %.1e, Hessian %.1e; %zu derivative evaluations, %zu from the "
            "ring\n",
            name,
            n_models,
            n_points,
            value_mismatch,
            worst_g,
            worst_h,
            worst_eg,
            worst_eh,
            evals,
            ring);
    }
    // (4) the face z = 0 and the edge y = 0, z = 1 of a second triangle: their bisector on the
    // planes x = const, 2 z - 1 = y^2, crosses the box (no offset side, as there)
    {
        Eigen::MatrixXd V(6, 3);
        V << -5, -5, 0, 5, -5, 0, 0, 5, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1;
        Eigen::MatrixXi F(2, 3);
        F << 0, 1, 2, 3, 4, 5;
        Input in;
        in.env = std::make_shared<Envelope>(true);
        in.env->init(V, F, 0.);
        in.pad = input_pad(V);
        in.eb = std::make_shared<ExactBandIntegral>(
            V,
            F,
            std::vector<int8_t>{},
            0,
            size_t(1024),
            in.env);
        const Vec3 c(0.5, -0.45, 0.6);
        const double w = 0.08;
        std::vector<Vec3> pts;
        std::vector<std::array<int, 4>> T;
        const Vec3 x0 = c + Vec3(0.01, -0.02, 0.015);
        box(c, w, x0, pts, T);
        std::vector<int> L(T.size(), 0);
        for (const int i : {0, 1, 4, 5, 8, 9}) L[size_t(i)] = 2; // sides z-, y-, x-
        Parameters param;
        auto mesh = make_mesh(param, pts, T, L, in);
        const auto ex = energy_at(*mesh, "exact");
        const auto sw = energy_at(*mesh, "exact_swept");
        REQUIRE(ex);
        REQUIRE(sw);
        double worst_g = 0., worst_h = 0.;
        for (const Vec3& dx :
             {Vec3(0., 0., 0.), Vec3(0.004, -0.003, 0.005), Vec3(-0.005, 0.002, -0.004)}) {
            Vec3 g, ge;
            Eigen::Matrix3d H, He;
            derivs(*sw, x0 + dx, g, H);
            derivs(*ex, x0 + dx, ge, He);
            worst_g = std::max(worst_g, (g - ge).norm() / ge.norm());
            worst_h = std::max(worst_h, (H - He).norm() / He.norm());
        }
        const size_t evals = mesh->m_swept_derivs.evals.exchange(0);
        const size_t ring = mesh->m_swept_derivs.ring_cells.exchange(0);
        CHECK(evals == 3);
        CHECK(ring == 3);
        CHECK(worst_g <= 1e-14);
        CHECK(worst_h <= 1e-14);
        std::printf(
            "  (4) curved bisector across the front faces: %zu derivative evaluations, %zu from "
            "the ring's band cells; against \"exact\" gradient %.1e, Hessian %.1e\n",
            evals,
            ring,
            worst_g,
            worst_h);
    }
}
