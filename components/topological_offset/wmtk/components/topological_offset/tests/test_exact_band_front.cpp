#include <wmtk/components/topological_offset/ExactBandIntegral.hpp>
#include <wmtk/components/topological_offset/OffsetPotential.hpp>

#include <catch2/catch_test_macros.hpp>

#include "exact_band_front_data.hpp"
#include "exact_band_integral_data.hpp"
#include "exact_band_side_data.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace wmtk::components::topological_offset;

/**
 * The front form of the band integral (EXPERIMENTAL_band_volume_rule "front_flux";
 * ExactBandIntegral::front_face(), bisector_jump(), prepare_front_flux(); BandVolumeEnergy3D's
 * set_front_flux()):
 *  1. against the Python prototype (exact_band_front_data.hpp, front_form.py / e_hessian.py,
 *     unsigned d): a boundary face's flux, gradient and Hessian terms, a cell's D_t;
 *  2. the identity oint_dt G.n dA + D_t = J_t (value()) on the stored band cells, with and
 *     without the offset side;
 *  3. the one-vertex energy: gradient and Hessian against central differences of the value and
 *     the gradient, symmetry, and equality with the "exact" rule's (the values differing by a
 *     constant), also where the boundary faces cross the input (cut edges);
 *  4. a curved bisector crossing a piece throws.
 */

namespace {

using Vec3 = Eigen::Vector3d;
using Tet = std::array<Vec3, 4>;
using Tri = std::array<Vec3, 3>;

/// The input envelope of the triangles (V, F), as TopoOffsetTetMesh::m_input_complex_envelope is
/// to the band term's searches (OffsetPotential.hpp). eps 0 builds no containment structure,
/// which no search reads.
std::shared_ptr<const wmtk::SampleEnvelope> triangle_envelope(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    auto env = std::make_shared<wmtk::SampleEnvelope>(true);
    env->init(V, F, 0.);
    return env;
}

Eigen::MatrixXd mat_V(const double* V, const int n)
{
    Eigen::MatrixXd M(n, 3);
    for (int i = 0; i < n; ++i) M.row(i) << V[3 * i], V[3 * i + 1], V[3 * i + 2];
    return M;
}
Eigen::MatrixXi mat_F(const int* F, const int n)
{
    Eigen::MatrixXi M(n, 3);
    for (int i = 0; i < n; ++i) M.row(i) << F[3 * i], F[3 * i + 1], F[3 * i + 2];
    return M;
}
Tet tet_of(const double* T)
{
    Tet t;
    for (int i = 0; i < 4; ++i) t[size_t(i)] = Vec3(T[3 * i], T[3 * i + 1], T[3 * i + 2]);
    return t;
}
double tet_vol(const Tet& T)
{
    return std::abs((T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]))) / 6.;
}

/// oint_dt G.n dA over t's four faces, each ordered with its normal out of t.
double boundary_flux(const ExactBandIntegral& eb, const Tet& T, const double delta)
{
    double tot = 0.;
    for (int i = 0; i < 4; ++i) {
        Tri f;
        int c = 0;
        for (int j = 0; j < 4; ++j) {
            if (j != i) f[size_t(c++)] = T[size_t(j)];
        }
        if ((f[1] - f[0]).cross(f[2] - f[0]).dot(f[0] - T[size_t(i)]) < 0.) std::swap(f[1], f[2]);
        tot += eb.front_face(f, delta);
    }
    return tot;
}

} // namespace

TEST_CASE("exact-band-front-python", "[offset][exact_band][front_flux]")
{
    // Parity with the prototype on boundary faces of two stored bands (cube_out, slot): with one
    // member, several members (resolve()), and planar bisector splits; and on flagged cells, D_t
    // zero and non-zero. Units: the flux and D_t against delta^2 times the area and delta times
    // the volume (their sizes), the gradient and Hessian relative.
    using namespace exact_band_front_data;
    std::printf("exact-band-front-python\n");
    double e_flux = 0., e_g = 0., e_h = 0., e_D = 0.;
    int n_multi = 0, n_split = 0;
    for (const FaceRef& r : faces) {
        const Case& c = cases[r.kase];
        const ExactBandIntegral eb(mat_V(c.V, c.nV), mat_F(c.F, c.nF));
        const Tri f = {
            {Vec3(r.P[0], r.P[1], r.P[2]),
             Vec3(r.P[3], r.P[4], r.P[5]),
             Vec3(r.P[6], r.P[7], r.P[8])}};
        const double area = 0.5 * (f[1] - f[0]).cross(f[2] - f[0]).norm();
        Vec3 G;
        Eigen::Matrix3d H;
        const double val = eb.front_face(f, c.delta);
        eb.front_face_derivs(f, c.delta, r.kv, G, H);
        const Eigen::Map<const Eigen::Vector3d> Gr(r.G);
        const Eigen::Matrix3d Hr =
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(r.H);
        INFO(c.name << " multi " << r.multi << " splits " << r.splits);
        const double ef = std::abs(val - r.flux) / (c.delta * c.delta * area);
        const double eg = (G - Gr).norm() / std::max(Gr.norm(), 1e-300);
        const double eh = (H - Hr).norm() / std::max(Hr.norm(), 1e-300);
        CHECK(ef <= 1e-12);
        CHECK(eg <= 1e-10);
        CHECK(eh <= 1e-10);
        ExactBandIntegral::FaceParts kp; // the kept parts give the same numbers
        CHECK(val == eb.front_face(f, c.delta, &kp));
        CHECK(val == eb.front_face(f, c.delta, &kp));
        Vec3 G2;
        Eigen::Matrix3d H2;
        eb.front_face_derivs(f, c.delta, r.kv, G2, H2, &kp);
        CHECK((G2 == G && H2 == H));
        e_flux = std::max(e_flux, ef);
        e_g = std::max(e_g, eg);
        e_h = std::max(e_h, eh);
        n_multi += r.multi > 0;
        n_split += r.splits > 0;
    }
    int n_nonzero = 0;
    for (const CellRef& r : cells) {
        const Case& c = cases[r.kase];
        const ExactBandIntegral eb(mat_V(c.V, c.nV), mat_F(c.F, c.nF));
        const Tet T = tet_of(r.T);
        const double D = eb.bisector_jump(T, c.delta);
        INFO(c.name << " D_ref " << r.D);
        const double e = std::abs(D - r.D) / (c.delta * tet_vol(T));
        CHECK(e <= 1e-12);
        if (r.D == 0.) CHECK(D == 0.);
        e_D = std::max(e_D, e);
        n_nonzero += r.D != 0.;
    }
    std::printf(
        "  %zu faces (%d with several members, %d with planar splits): flux %.1e (of delta^2 "
        "area), "
        "gradient %.1e, Hessian %.1e | %zu cells (%d with D_t != 0): D_t %.1e (of delta vol)\n",
        std::size(faces),
        n_multi,
        n_split,
        e_flux,
        e_g,
        e_h,
        std::size(cells),
        n_nonzero,
        e_D);
}

TEST_CASE("exact-band-front-identity", "[offset][exact_band][front_flux]")
{
    // oint_dt G.n dA + D_t = J_t = value(t) on every stored cell where value() is exact (no
    // rule-(a) piece): the divergence theorem, which is why the operations under "front_flux"
    // read value() (TopoOffsetTetMesh::band_cell_term()). Without the side on the five real cases
    // of exact_band_integral_data.hpp, with it on the side cases of exact_band_side_data.hpp.
    std::printf("exact-band-front-identity: oint G.n + D_t against value()\n");
    int total = 0;
    const auto run = [](const char* name,
                        const ExactBandIntegral& eb,
                        const std::vector<Tet>& cells,
                        const std::vector<double>& deltas) {
        double worst = 0.;
        int n = 0, rule_a = 0, curved = 0, with_D = 0;
        for (size_t i = 0; i < cells.size(); ++i) {
            const Tet& T = cells[i];
            const double delta = deltas[i];
            ExactBandIntegral::Stats st;
            const double J = eb.value(T, delta, &st);
            if (st.rule_a_vol > 0.) {
                ++rule_a;
                continue;
            }
            double F, D;
            try {
                F = boundary_flux(eb, T, delta);
                D = eb.bisector_jump(T, delta);
            } catch (const std::runtime_error&) {
                ++curved;
                continue;
            }
            const double e = std::abs(F + D - J) / (delta * tet_vol(T));
            INFO(name << " cell " << i << " J " << J << " flux " << F << " D " << D);
            CHECK(e <= 1e-11);
            worst = std::max(worst, e);
            ++n;
            with_D += D != 0.;
        }
        std::printf(
            "  %-13s %d cells: max |oint + D - J| / (delta vol) %.1e (%d with D_t != 0) | skipped: "
            "%d with rule-(a) pieces, %d curved\n",
            name,
            n,
            worst,
            with_D,
            rule_a,
            curved);
        return n;
    };
    for (const auto& c : exact_band_data::real_cases) {
        const ExactBandIntegral eb(mat_V(c.V, c.nV), mat_F(c.F, c.nF));
        std::vector<Tet> cells;
        for (int i = 0; i < c.ncells; ++i) cells.push_back(tet_of(c.cells[i].T));
        total += run(c.name, eb, cells, std::vector<double>(cells.size(), c.delta));
    }
    for (const auto& c : exact_band_side_data::side_cases) {
        const ExactBandIntegral eb(
            mat_V(c.V, c.nV),
            mat_F(c.F, c.nF),
            std::vector<int8_t>(size_t(c.nF), 1),
            c.sigma);
        std::vector<Tet> cells;
        std::vector<double> deltas;
        for (int i = 0; i < c.ncells; ++i) {
            cells.push_back(tet_of(c.cells[i].T));
            deltas.push_back(c.cells[i].delta);
        }
        total += run((std::string(c.name) + " side").c_str(), eb, cells, deltas);
    }
    CHECK(total > 0);
}

namespace {

/// A ring of four cells under x around ctr and its boundary faces through x (those in one cell
/// only, the top four), oriented out of the cells, as smooth_front_vertex() builds them.
struct Ring
{
    std::vector<BandVolumeEnergy3D::Cell> cells;
    std::vector<BandVolumeEnergy3D::Face> faces;
};
Ring ring_at(const Vec3& ctr, const Vec3& dx, const Vec3& dy, const Vec3& dz)
{
    Ring r;
    for (const double sx : {-1., 1.}) {
        for (const double sy : {-1., 1.}) {
            BandVolumeEnergy3D::Cell cl;
            cl.q1 = ctr + sx * dx;
            cl.q2 = ctr + sy * dy;
            cl.q3 = ctr - dz;
            r.cells.push_back(cl);
            BandVolumeEnergy3D::Face f{cl.q1, cl.q2};
            if ((f.a - ctr).cross(f.b - ctr).dot(cl.q3 - ctr) > 0.) std::swap(f.a, f.b);
            r.faces.push_back(f);
        }
    }
    return r;
}

/// The front-flux energy at the ring against central differences and the "exact" energy.
void check_energy(
    const char* name,
    const std::shared_ptr<const ExactBandIntegral>& eb,
    const std::shared_ptr<const wmtk::SampleEnvelope>& tris,
    const double pad,
    const Ring& r,
    const Vec3& ctr,
    const Vec3& x,
    const double delta)
{
    BandVolumeEnergy3D ff(tris, pad, r.cells, ctr, delta), ex(tris, pad, r.cells, ctr, delta);
    ff.set_front_flux(eb, delta, r.faces);
    ex.set_exact_integral(eb, delta);
    const Eigen::VectorXd xv = x;
    Eigen::VectorXd g, ge;
    Eigen::MatrixXd H, He;
    ff.gradient(xv, g);
    ff.hessian(xv, H);
    ex.gradient(xv, ge);
    ex.hessian(xv, He);
    const double asym = (H - H.transpose()).norm() / H.norm();
    const double vs_g = (g - ge).norm() / ge.norm(), vs_h = (H - He).norm() / He.norm();
    // the values differ by the flux through the faces that do not hold x: a constant
    const Eigen::VectorXd x2 = xv + Vec3(0.0011, -0.0007, 0.0013);
    const double dff = ff.value(x2) - ff.value(xv), dex = ex.value(x2) - ex.value(xv);
    const double vs_v = std::abs(dff - dex) / std::abs(dex);
    double fd_g = std::numeric_limits<double>::infinity(), fd_h = fd_g;
    for (const double h : {1e-5, 1e-6, 1e-7}) {
        Vec3 gf;
        Eigen::Matrix3d Hf;
        for (int k = 0; k < 3; ++k) {
            Eigen::VectorXd xp = xv, xm = xv;
            xp[k] += h;
            xm[k] -= h;
            gf[k] = (ff.value(xp) - ff.value(xm)) / (2. * h);
            Eigen::VectorXd gp, gm;
            ff.gradient(xp, gp);
            ff.gradient(xm, gm);
            Hf.col(k) = (gp - gm) / (2. * h);
        }
        fd_g = std::min(fd_g, (gf - Vec3(g)).norm() / g.norm());
        fd_h = std::min(fd_h, (Hf - Eigen::Matrix3d(H)).norm() / H.norm());
    }
    std::printf(
        "  %-26s FD (min over h): gradient %.1e Hessian %.1e | asymmetry %.1e | vs \"exact\": "
        "gradient %.1e Hessian %.1e, value differences %.1e\n",
        name,
        fd_g,
        fd_h,
        asym,
        vs_g,
        vs_h,
        vs_v);
    INFO(name);
    CHECK(fd_g <= 1e-7);
    CHECK(fd_h <= 1e-6);
    CHECK(asym <= 1e-10);
    CHECK(vs_g <= 1e-10);
    CHECK(vs_h <= 1e-9);
    CHECK(vs_v <= 1e-9);
}

} // namespace

TEST_CASE("exact-band-front-energy", "[offset][exact_band][front_flux]")
{
    // BandVolumeEnergy3D under "front_flux" on rings around points of the stored cube's band:
    // beyond an edge (edge and face labels), near a corner (vertex labels), and astride a cube
    // face (the boundary faces cross the input: cut edges, with the offset side). Each against
    // central differences and against the "exact" rule on the same ring.
    std::printf("exact-band-front-energy\n");
    const auto& c = exact_band_data::real_cases[0];
    REQUIRE(std::string(c.name) == "cube_out");
    const Eigen::MatrixXd V = mat_V(c.V, c.nV);
    const Eigen::MatrixXi F = mat_F(c.F, c.nF);
    const auto plain = std::make_shared<const ExactBandIntegral>(V, F);
    const auto& sc = exact_band_side_data::side_cases[0];
    REQUIRE(std::string(sc.name) == "cube_out");
    const auto sided = std::make_shared<const ExactBandIntegral>(
        mat_V(sc.V, sc.nV),
        mat_F(sc.F, sc.nF),
        std::vector<int8_t>(size_t(sc.nF), 1),
        sc.sigma);
    const auto tris = triangle_envelope(V, F);
    const double pad = input_pad(V);
    const double delta = c.delta;
    const Vec3 X = Vec3::UnitX(), Y = Vec3::UnitY(), Z = Vec3::UnitZ();
    {
        const Vec3 ctr(0.52, 0.1, 0.515); // beyond the edge x = z = 0.5
        const Ring r = ring_at(ctr, 0.02 * X, 0.02 * Y, 0.02 * Z);
        check_energy(
            "edge, no side",
            plain,
            tris,
            pad,
            r,
            ctr,
            ctr + Vec3(0.002, 0.001, 0.004),
            delta);
        check_energy(
            "edge, side",
            sided,
            tris,
            pad,
            r,
            ctr,
            ctr + Vec3(0.002, 0.001, 0.004),
            delta);
    }
    {
        const Vec3 ctr(0.515, 0.52, 0.51); // beyond the corner (0.5, 0.5, 0.5)
        const Ring r = ring_at(ctr, 0.015 * X, 0.015 * Y, 0.015 * Z);
        check_energy(
            "corner, side",
            sided,
            tris,
            pad,
            r,
            ctr,
            ctr + Vec3(0.001, -0.002, 0.001),
            delta);
    }
    {
        // x just inside the face z = 0.5, the ring's other corners above it: every boundary face
        // crosses the input's plane
        const Vec3 ctr(0.1, 0.05, 0.496);
        const Ring r = ring_at(ctr, 0.02 * X + 0.012 * Z, 0.02 * Y + 0.012 * Z, 0.02 * Z);
        check_energy(
            "astride a face, side",
            sided,
            tris,
            pad,
            r,
            ctr,
            ctr + Vec3(0.001, 0.0005, -0.0005),
            delta);
    }
}

TEST_CASE("exact-band-front-curved", "[offset][exact_band][front_flux]")
{
    // A face (z = 0) and the edge y = 0, z = 1 of a second triangle: on the plane x = 0.5,
    // y < 0, both are members, and d_face = z against d_edge = sqrt(y^2 + (z - 1)^2): their
    // bisector, 2 z - 1 = y^2, is curved. A triangle and a cell it crosses throw; a triangle
    // wholly nearer the face does not.
    Eigen::MatrixXd V(6, 3);
    V << -5, -5, 0, 5, -5, 0, 0, 5, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1;
    Eigen::MatrixXi F(2, 3);
    F << 0, 1, 2, 3, 4, 5;
    const ExactBandIntegral eb(V, F);
    const Tri f = {{Vec3(0.5, -1., 0.2), Vec3(0.5, -0.1, 0.2), Vec3(0.5, -0.5, 0.9)}};
    CHECK_THROWS_AS(eb.front_face(f, 0.3), std::runtime_error);
    const Tet T = {{f[0], f[1], f[2], Vec3(0.6, -0.5, 0.5)}};
    CHECK_THROWS_AS(eb.bisector_jump(T, 0.3), std::runtime_error);
    std::string msg;
    try {
        eb.front_face(f, 0.3);
    } catch (const std::runtime_error& e) {
        msg = e.what();
    }
    std::printf("exact-band-front-curved: %s\n", msg.substr(0, 160).c_str());
    CHECK(msg.find("curved") != std::string::npos);
    const Tri g = {{Vec3(0.5, -1., 0.05), Vec3(0.5, -0.9, 0.05), Vec3(0.5, -0.95, 0.1)}};
    CHECK_NOTHROW(eb.front_face(g, 0.3));
}

/**
 * Hidden benchmark: FRONT_FLUX_BENCH_FILE=<band>.bin ./wmtk_test_topological_offset
 * "[.bench_front]". Reads a stored band (int32 nV, nF, nT; double delta; V; F; the band cells,
 * 12 doubles each; the band outside the input), and for every front vertex (a vertex of a
 * boundary face of the band, off the input) times ONE evaluation of the band part of the front
 * objective -- value, gradient and Hessian at the vertex's position, on a fresh
 * BandVolumeEnergy3D -- under "corner_bound", "exact" (no value table: each value computed) and
 * "front_flux". Also: front_flux's gradient and Hessian against exact's at every vertex, and the
 * flag against D_t on every band cell. FF_BENCH_SPLIT=1 splits front_flux's time (labelling,
 * flux, derivatives); FF_BENCH_DEBUG=1 prints the vertices where the two disagree, with central
 * differences of the gradient against each Hessian. FF_BENCH_DUMP=<file> writes front_flux's
 * results bit for bit, to compare two builds: per front vertex the value alone, then value,
 * gradient, Hessian, then the number of boundary faces through it and per face front_face(),
 * then front_face_derivs()' gradient and Hessian (doubles).
 */
TEST_CASE("exact-band-front-bench", "[.bench_front]")
{
    const char* file = std::getenv("FRONT_FLUX_BENCH_FILE");
    if (!file) {
        WARN("FRONT_FLUX_BENCH_FILE not set");
        return;
    }
    std::ifstream in(file, std::ios::binary);
    REQUIRE(in.good());
    int32_t hdr[3];
    double delta;
    in.read(reinterpret_cast<char*>(hdr), sizeof(hdr));
    in.read(reinterpret_cast<char*>(&delta), sizeof(delta));
    const int nV = hdr[0], nF = hdr[1], nT = hdr[2];
    std::vector<double> Vb(size_t(3 * nV)), Tb(size_t(12 * nT));
    std::vector<int32_t> Fb(size_t(3 * nF));
    in.read(reinterpret_cast<char*>(Vb.data()), std::streamsize(Vb.size() * sizeof(double)));
    in.read(reinterpret_cast<char*>(Fb.data()), std::streamsize(Fb.size() * sizeof(int32_t)));
    in.read(reinterpret_cast<char*>(Tb.data()), std::streamsize(Tb.size() * sizeof(double)));
    REQUIRE(in.good());
    const Eigen::MatrixXd V = mat_V(Vb.data(), nV);
    Eigen::MatrixXi F(nF, 3);
    for (int i = 0; i < nF; ++i)
        F.row(i) << Fb[size_t(3 * i)], Fb[size_t(3 * i + 1)], Fb[size_t(3 * i + 2)];
    // the solid's side: the triangles' orientation (signed volume), the band outside it
    double vol6 = 0.;
    for (int i = 0; i < nF; ++i) {
        const Vec3 a = V.row(F(i, 0)), b = V.row(F(i, 1)), c = V.row(F(i, 2));
        vol6 += a.dot(b.cross(c));
    }
    const std::vector<int8_t> side(size_t(nF), vol6 > 0. ? 1 : -1);
    auto eb_m = std::make_shared<ExactBandIntegral>(V, F, side, 1);
    eb_m->prepare_front_flux(delta);
    const std::shared_ptr<const ExactBandIntegral> eb = eb_m;
    const auto tris = triangle_envelope(V, F);
    const double pad = input_pad(V);

    // unique vertices (exact coordinates), cells as vertex ids
    std::vector<Vec3> P;
    std::vector<std::array<int, 4>> cell(static_cast<size_t>(nT));
    {
        std::vector<std::pair<std::array<double, 3>, int>> pts;
        for (int t = 0; t < nT; ++t) {
            for (int j = 0; j < 4; ++j) {
                const double* p = &Tb[size_t(12 * t + 3 * j)];
                pts.push_back({{p[0], p[1], p[2]}, 4 * t + j});
            }
        }
        std::sort(pts.begin(), pts.end());
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i == 0 || pts[i].first != pts[i - 1].first) {
                P.push_back(Vec3(pts[i].first[0], pts[i].first[1], pts[i].first[2]));
            }
            cell[size_t(pts[i].second / 4)][size_t(pts[i].second % 4)] = int(P.size()) - 1;
        }
    }
    // boundary faces of the band: faces of exactly one cell, oriented out of it
    std::map<std::array<int, 3>, std::vector<int>> face_cells;
    for (int t = 0; t < nT; ++t) {
        for (int i = 0; i < 4; ++i) {
            std::array<int, 3> k;
            int c = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != i) k[size_t(c++)] = cell[size_t(t)][size_t(j)];
            }
            std::sort(k.begin(), k.end());
            face_cells[k].push_back(t);
        }
    }
    std::vector<std::vector<int>> ring(P.size()), bfaces(P.size());
    std::vector<std::array<int, 4>> bf; // the face's three vertices (out of the cell), its cell
    for (int t = 0; t < nT; ++t) {
        for (const int v : cell[size_t(t)]) ring[size_t(v)].push_back(t);
    }
    for (const auto& [k, ts] : face_cells) {
        if (ts.size() != 1) continue;
        const auto& c = cell[size_t(ts[0])];
        int z = -1;
        for (const int u : c) {
            if (u != k[0] && u != k[1] && u != k[2]) z = u;
        }
        std::array<int, 4> f = {k[0], k[1], k[2], ts[0]};
        if ((P[size_t(f[1])] - P[size_t(f[0])])
                .cross(P[size_t(f[2])] - P[size_t(f[0])])
                .dot(P[size_t(z)] - P[size_t(f[0])]) > 0.) {
            std::swap(f[1], f[2]);
        }
        for (int j = 0; j < 3; ++j) bfaces[size_t(f[size_t(j)])].push_back(int(bf.size()));
        bf.push_back(f);
    }
    std::vector<int> front;
    for (size_t v = 0; v < P.size(); ++v) {
        // the outer boundary (the front at about delta), not the inner one on the input
        if (!bfaces[v].empty() && eb->distance(P[v]) > 0.5 * delta) front.push_back(int(v));
    }
    std::printf(
        "exact-band-front-bench: %d band cells, %zu boundary faces, %zu front vertices (d > "
        "delta / 2); front_flux: %zu flag polytopes (reach %.4g)\n",
        nT,
        bf.size(),
        front.size(),
        eb->front_flux_polytopes(),
        eb->front_flux_reach());

    std::FILE* dump = nullptr;
    if (const char* df = std::getenv("FF_BENCH_DUMP")) dump = std::fopen(df, "wb");
    const auto put = [&](const double* a, const size_t n) {
        if (dump) std::fwrite(a, sizeof(double), n, dump);
    };
    using clk = std::chrono::steady_clock;
    std::vector<double> t_cb, t_ex, t_ff, t_ff_der, t_split_v, t_split_d, t_split_p, t_split_f;
    std::vector<double> v_cb, v_ex, v_ff; // value only
    double worst_g = 0., worst_h = 0.;
    std::vector<double> eg_all, eh_all;
    int n_D = 0, n_D_max = 0, n_curved = 0;
    const auto cells_of = [&](const int v) {
        std::vector<BandVolumeEnergy3D::Cell> cells;
        for (const int t : ring[size_t(v)]) {
            BandVolumeEnergy3D::Cell c;
            int k = 0;
            for (const int u : cell[size_t(t)]) {
                if (u == v) continue;
                (k == 0 ? c.q1 : k == 1 ? c.q2 : c.q3) = P[size_t(u)];
                ++k;
            }
            cells.push_back(c);
        }
        return cells;
    };
    const auto faces_of = [&](const int v) {
        std::vector<BandVolumeEnergy3D::Face> faces;
        for (const int fi : bfaces[size_t(v)]) {
            const auto& f = bf[size_t(fi)];
            int j = 0;
            while (f[size_t(j)] != v) ++j;
            faces.push_back({P[size_t(f[size_t((j + 1) % 3)])], P[size_t(f[size_t((j + 2) % 3)])]});
        }
        return faces;
    };
    // The value alone, in a pass of its own: the closed forms keep per-thread tables of their
    // last results (ExactBandIntegral.cpp, jXi()), which a value at the same x just before would
    // fill for the evaluation timed below.
    std::vector<double> vo_all;
    for (const int v : front) {
        const Vec3 x = P[size_t(v)];
        const auto cells = cells_of(v);
        const auto faces = faces_of(v);
        const Eigen::VectorXd xv = x;
        const auto timed_value = [&](BandVolumeEnergy3D& e, std::vector<double>& out) {
            const auto t0 = clk::now();
            e.value(xv);
            out.push_back(std::chrono::duration<double, std::micro>(clk::now() - t0).count());
        };
        BandVolumeEnergy3D e(tris, pad, cells, x, delta);
        timed_value(e, v_cb);
        BandVolumeEnergy3D e2(tris, pad, cells, x, delta);
        e2.set_exact_integral(eb, delta);
        timed_value(e2, v_ex);
        BandVolumeEnergy3D e3(tris, pad, cells, x, delta);
        e3.set_front_flux(eb, delta, faces);
        double vo = std::numeric_limits<double>::quiet_NaN();
        try {
            timed_value(e3, v_ff);
            vo = e3.value(xv);
        } catch (const std::runtime_error&) {
        }
        vo_all.push_back(vo);
    }
    // value, gradient and Hessian at xv, timed into out; the derivatives' part into der
    const auto timed = [&](BandVolumeEnergy3D& e,
                           const Eigen::VectorXd& xv,
                           Eigen::VectorXd& g,
                           Eigen::MatrixXd& H,
                           std::vector<double>& out,
                           double& der) {
        const auto t0 = clk::now();
        const double val = e.value(xv);
        const auto t1 = clk::now();
        e.gradient(xv, g);
        e.hessian(xv, H);
        const auto t2 = clk::now();
        out.push_back(std::chrono::duration<double, std::micro>(t2 - t0).count());
        der = std::chrono::duration<double, std::micro>(t2 - t1).count();
        return val;
    };
    // The "exact" rule's evaluations in a pass of their own as well: at the same vertex they fill
    // the tables front_flux's read (MEASURED, cube_out: front_flux 48.8 us after them, 82.3 us
    // without).
    std::vector<Eigen::VectorXd> ge_all;
    std::vector<Eigen::MatrixXd> He_all;
    for (const int v : front) {
        const Eigen::VectorXd xv = P[size_t(v)];
        Eigen::VectorXd g;
        Eigen::MatrixXd H;
        double der = 0.;
        BandVolumeEnergy3D e(tris, pad, cells_of(v), P[size_t(v)], delta);
        e.set_exact_integral(eb, delta);
        timed(e, xv, g, H, t_ex, der);
        ge_all.push_back(g);
        He_all.push_back(H);
    }
    for (size_t iv = 0; iv < front.size(); ++iv) {
        const int v = front[iv];
        const Vec3 x = P[size_t(v)];
        const auto cells = cells_of(v);
        const auto faces = faces_of(v);
        const Eigen::VectorXd xv = x;
        Eigen::VectorXd g;
        Eigen::MatrixXd H;
        put(&vo_all[iv], 1);
        double der = 0.;
        {
            BandVolumeEnergy3D e(tris, pad, cells, x, delta);
            timed(e, xv, g, H, t_cb, der);
        }
        const Eigen::VectorXd& ge = ge_all[iv];
        const Eigen::MatrixXd& He = He_all[iv];
        try {
            BandVolumeEnergy3D e(tris, pad, cells, x, delta);
            e.set_front_flux(eb, delta, faces);
            const double val = timed(e, xv, g, H, t_ff, der);
            t_ff_der.push_back(der);
            put(&val, 1);
            put(g.data(), 3);
            put(H.data(), 9);
        } catch (const std::runtime_error&) {
            ++n_curved;
            continue;
        }
        const double nf = dump ? double(faces.size()) : 0.;
        put(&nf, 1);
        for (size_t fi = 0; fi < size_t(nf); ++fi) {
            const auto& f = faces[fi];
            const double fv = eb->front_face({{x, f.a, f.b}}, delta);
            Vec3 gg;
            Eigen::Matrix3d hh;
            eb->front_face_derivs({{x, f.a, f.b}}, delta, 0, gg, hh);
            put(&fv, 1);
            put(gg.data(), 3);
            put(hh.data(), 9);
        }
        if (std::getenv("FF_BENCH_SPLIT")) {
            const auto t0 = clk::now();
            for (const auto& f : faces) eb->front_face({{x, f.a, f.b}}, delta);
            const auto t1 = clk::now();
            for (const auto& f : faces) {
                Vec3 gg;
                Eigen::Matrix3d hh;
                eb->front_face_derivs({{x, f.a, f.b}}, delta, 0, gg, hh);
            }
            const auto t2 = clk::now();
            std::vector<ExactBandIntegral::FaceParts> kp(faces.size());
            for (size_t i = 0; i < faces.size(); ++i)
                eb->front_face({{x, faces[i].a, faces[i].b}}, delta, &kp[i]);
            const auto t3 = clk::now();
            for (size_t i = 0; i < faces.size(); ++i)
                eb->front_face({{x, faces[i].a, faces[i].b}}, delta, &kp[i]);
            const auto t4 = clk::now();
            t_split_v.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            t_split_d.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
            t_split_p.push_back(
                std::chrono::duration<double, std::micro>((t3 - t2) - (t4 - t3)).count());
            t_split_f.push_back(std::chrono::duration<double, std::micro>(t4 - t3).count());
        }
        const double eg = (g - ge).norm() / std::max(ge.norm(), 1e-300);
        const double eh = (H - He).norm() / std::max(He.norm(), 1e-300);
        eg_all.push_back(eg);
        eh_all.push_back(eh);
        if (std::getenv("FF_BENCH_DEBUG") && (eg > 1e-8 || eh > 1e-8)) {
            double dmin = 1e300;
            for (const auto& c : cells)
                dmin = std::min({dmin, eb->distance(c.q1), eb->distance(c.q2), eb->distance(c.q3)});
            // central differences of the two gradients (equal to rounding here)
            double hfd_ff = 1e300, hfd_ex = 1e300;
            for (const double hh : {1e-5, 1e-6, 1e-7}) {
                Eigen::Matrix3d Hf, He2;
                for (int k = 0; k < 3; ++k) {
                    Eigen::VectorXd gp, gm;
                    BandVolumeEnergy3D a(tris, pad, cells, x, delta), b(tris, pad, cells, x, delta);
                    a.set_front_flux(eb, delta, faces);
                    b.set_exact_integral(eb, delta);
                    Eigen::VectorXd xp = xv, xm = xv;
                    xp[k] += hh;
                    xm[k] -= hh;
                    a.gradient(xp, gp);
                    a.gradient(xm, gm);
                    Hf.col(k) = (gp - gm) / (2 * hh);
                    b.gradient(xp, gp);
                    b.gradient(xm, gm);
                    He2.col(k) = (gp - gm) / (2 * hh);
                }
                hfd_ff = std::min(hfd_ff, (Hf - Eigen::Matrix3d(H)).norm() / H.norm());
                hfd_ex = std::min(hfd_ex, (He2 - Eigen::Matrix3d(He)).norm() / He.norm());
            }
            std::printf(
                "    FD of the gradient vs H: front_flux %.1e, exact %.1e\n",
                hfd_ff,
                hfd_ex);
            std::printf(
                "    outlier v %d d(x)/delta %.4f min corner d/delta %.3e cells %zu faces %zu | g "
                "(%.3e %.3e %.3e) exact (%.3e %.3e %.3e) | eg %.1e eh %.1e\n",
                v,
                eb->distance(x) / delta,
                dmin / delta,
                cells.size(),
                faces.size(),
                g[0],
                g[1],
                g[2],
                ge[0],
                ge[1],
                ge[2],
                eg,
                eh);
        }
        worst_g = std::max(worst_g, eg);
        worst_h = std::max(worst_h, eh);
        // D_t evaluations this vertex's value made (the energy's own rule)
        int nd = 0;
        for (const auto& c : cells) {
            const Tet t = {{x, c.q1, c.q2, c.q3}};
            nd += eb->distance_bound(t) > eb->front_flux_reach() || eb->front_flux_flag(t);
        }
        n_D += nd;
        n_D_max = std::max(n_D_max, nd);
    }
    const auto q = [](std::vector<double> v, const double p) {
        if (v.empty()) return 0.;
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, size_t(p * double(v.size())))];
    };
    std::printf(
        "  one evaluation of the band part (value + gradient + Hessian), us, median / p90 / max:\n"
        "    corner_bound %.1f / %.1f / %.1f\n    exact        %.1f / %.1f / %.1f\n"
        "    front_flux   %.1f / %.1f / %.1f (of which gradient + Hessian %.1f / %.1f)\n",
        q(t_cb, 0.5),
        q(t_cb, 0.9),
        q(t_cb, 1.),
        q(t_ex, 0.5),
        q(t_ex, 0.9),
        q(t_ex, 1.),
        q(t_ff, 0.5),
        q(t_ff, 0.9),
        q(t_ff, 1.),
        q(t_ff_der, 0.5),
        q(t_ff_der, 0.9));
    std::printf(
        "  the value alone (a line-search trial), us, median / p90: corner_bound %.1f / %.1f | "
        "exact %.1f / %.1f | front_flux %.1f / %.1f\n",
        q(v_cb, 0.5),
        q(v_cb, 0.9),
        q(v_ex, 0.5),
        q(v_ex, 0.9),
        q(v_ff, 0.5),
        q(v_ff, 0.9));
    if (!t_split_v.empty()) {
        std::printf(
            "  front_flux split, median / p90 us: faces value only %.1f / %.1f, faces with "
            "derivatives %.1f / %.1f; of the value: labelling %.1f / %.1f, flux %.1f / %.1f\n",
            q(t_split_v, 0.5),
            q(t_split_v, 0.9),
            q(t_split_d, 0.5),
            q(t_split_d, 0.9),
            q(t_split_p, 0.5),
            q(t_split_p, 0.9),
            q(t_split_f, 0.5),
            q(t_split_f, 0.9));
    }
    std::printf(
        "  front_flux vs exact, %zu vertices: gradient median %.1e max %.1e, Hessian median %.1e "
        "max %.1e | D_t evaluations per vertex: mean %.2f max %d | curved throws %d\n",
        eg_all.size(),
        q(eg_all, 0.5),
        worst_g,
        q(eh_all, 0.5),
        worst_h,
        double(n_D) / std::max<size_t>(1, eg_all.size()),
        n_D_max,
        n_curved);
    // the flag against D_t on every band cell
    int flagged = 0, uncertified = 0, nonzero = 0, missed = 0, curved_cells = 0;
    double missed_max = 0.;
    const auto t0 = clk::now();
    for (int t = 0; t < nT; ++t) {
        const Tet T = tet_of(&Tb[size_t(12 * t)]);
        const bool unc = eb->distance_bound(T) > eb->front_flux_reach();
        const bool fl = eb->front_flux_flag(T);
        uncertified += unc;
        flagged += fl;
        double D;
        try {
            D = eb->bisector_jump(T, delta);
        } catch (const std::runtime_error&) {
            ++curved_cells;
            continue;
        }
        if (D != 0.) {
            ++nonzero;
            if (!fl && !unc) {
                const double rel = std::abs(D) / (delta * tet_vol(T));
                missed += rel > 1e-12; // beyond rounding
                missed_max = std::max(missed_max, rel);
            }
        }
    }
    std::printf(
        "  flag, every band cell: %d flagged by the polytopes, %d beyond the reach; D_t != 0 on "
        "%d; "
        "neither flagged nor beyond: %d over 1e-12 delta vol (max |D_t| / (delta vol) %.1e); %d "
        "curved throws (%.1f s)\n",
        flagged,
        uncertified,
        nonzero,
        missed,
        missed_max,
        curved_cells,
        std::chrono::duration<double>(clk::now() - t0).count());
    if (dump) std::fclose(dump);
    CHECK(missed == 0);
}
