#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/tracking/SurfaceMap.hpp>
#include <wmtk/components/tracking/segment_walk.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <map>
#include <random>
#include <set>
#include <string>

// Cases too slow for the Debug CI jobs: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace wmtk::components;
using tracking::SurfaceCurve;
using tracking::SurfaceMap;
using tracking::SurfacePoint;
using tracking::SurfaceSegment;
using tracking::Triangle2;
using tracking::WalkPiece;
using tracking::WalkPoint;

namespace {

/// A square grid of n by n cells, each cut along the same diagonal.
std::vector<Triangle2> grid(const int n)
{
    std::vector<Triangle2> tris;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const Eigen::Vector2d a(i, j), b(i + 1, j), c(i + 1, j + 1), d(i, j + 1);
            tris.push_back({{a, b, c}});
            tris.push_back({{a, c, d}});
        }
    }
    return tris;
}

Eigen::Vector2d at(const Triangle2& t, const Eigen::Vector3d& l)
{
    return l[0] * t[0] + l[1] * t[1] + l[2] * t[2];
}

/// The triangle of the grid holding (x, y) and its coordinates there.
WalkPoint locate(const std::vector<Triangle2>& tris, const Eigen::Vector2d& q)
{
    for (size_t t = 0; t < tris.size(); ++t) {
        const auto& c = tris[t];
        const Eigen::Vector2d e0 = c[1] - c[0], e1 = c[2] - c[0], p = q - c[0];
        const double det = e0.x() * e1.y() - e0.y() * e1.x();
        const double l1 = (p.x() * e1.y() - p.y() * e1.x()) / det;
        const double l2 = (e0.x() * p.y() - e0.y() * p.x()) / det;
        const double l0 = 1 - l1 - l2;
        if (l0 >= 0 && l1 >= 0 && l2 >= 0) return {t, Eigen::Vector3d(l0, l1, l2)};
    }
    return {0, Eigen::Vector3d::Zero()};
}

/// That the pieces run from `p` to `q` along the straight line, end to end, in order.
void check_walk(
    const std::vector<Triangle2>& tris,
    const std::vector<WalkPiece>& pieces,
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& q)
{
    const Eigen::Vector2d d = (q - p).normalized();
    REQUIRE(!pieces.empty());
    CHECK((at(tris[pieces.front().tri], pieces.front().bary[0]) - p).norm() < 1e-12);
    CHECK((at(tris[pieces.back().tri], pieces.back().bary[1]) - q).norm() < 1e-12);
    double along = 0;
    Eigen::Vector2d last = p;
    for (const WalkPiece& w : pieces) {
        const Eigen::Vector2d a = at(tris[w.tri], w.bary[0]), b = at(tris[w.tri], w.bary[1]);
        CHECK((a - last).norm() < 1e-12);
        for (const auto& x : {a, b}) {
            const Eigen::Vector2d e = x - p;
            CHECK(std::abs(e.x() * d.y() - e.y() * d.x()) < 1e-12);
            CHECK(e.dot(d) >= along - 1e-12);
            along = std::max(along, e.dot(d));
        }
        CHECK(w.bary[0].minCoeff() >= 0);
        CHECK(w.bary[1].minCoeff() >= 0);
        last = b;
    }
}

history::SurfaceHistory remesh(const std::string& model, double len_factor, int its)
{
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/" + model, V, F, 0);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);
    isotropic_remeshing::IsotropicRemeshing m(verts, 0);
    m.create_mesh(verts.size(), tris, {}, true, 0, 0);
    const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
    m.uniform_remeshing(len_factor * m.average_len_valen()[0], its);
    m.consolidate_mesh();
    m.stop_history();
    return rec->history();
}

Eigen::Vector3d position(
    const uint64_t face,
    const Eigen::Vector3d& b,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F)
{
    const auto& f = F[face];
    return b[0] * V[f[0]] + b[1] * V[f[1]] + b[2] * V[f[2]];
}

/// A remeshing of the unit sphere that keeps the curve between the faces above and below `z` as a
/// feature, so its collapses and moves along the curve split triangles virtually.
history::SurfaceHistory remesh_with_curve(const double z, double len_factor, int its)
{
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/sphere.obj", V, F, 0);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e300), hi = -lo;
    for (const auto& p : verts) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    const double cut = lo.z() + z * (hi.z() - lo.z());
    std::map<std::pair<size_t, size_t>, std::vector<bool>> sides;
    for (const auto& t : tris) {
        const bool above = (verts[t[0]].z() + verts[t[1]].z() + verts[t[2]].z()) / 3 > cut;
        for (int k = 0; k < 3; ++k) {
            sides[std::minmax(t[k], t[(k + 1) % 3])].push_back(above);
        }
    }
    std::vector<std::array<size_t, 2>> features;
    for (const auto& [e, on] : sides) {
        if (on.size() == 2 && on[0] != on[1]) features.push_back({{e.first, e.second}});
    }
    REQUIRE(!features.empty());
    isotropic_remeshing::IsotropicRemeshing m(verts, 0);
    m.create_mesh(verts.size(), tris, {}, true, 0, 0);
    m.set_feature_edges(features);
    const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
    for (const auto& e : m.feature_edges()) rec->history().features.push_back({{e[0], e[1]}});
    m.uniform_remeshing(len_factor * m.average_len_valen()[0], its);
    m.consolidate_mesh();
    m.stop_history();
    return rec->history();
}

/// A curve across faces: from inside a face, over a random edge to the next face, and on.
SurfaceCurve random_curve(
    const std::vector<std::array<uint64_t, 3>>& F,
    const std::vector<uint8_t>& alive,
    const int length,
    std::mt19937& rng)
{
    std::map<std::pair<uint64_t, uint64_t>, std::vector<uint64_t>> on_edge;
    for (uint64_t f = 0; f < F.size(); ++f) {
        if (!alive[f]) continue;
        for (int k = 0; k < 3; ++k) {
            const auto [a, b] = std::minmax(F[f][k], F[f][(k + 1) % 3]);
            on_edge[{a, b}].push_back(f);
        }
    }
    std::uniform_real_distribution<double> u(0.05, 0.95);
    std::vector<uint64_t> live;
    for (uint64_t f = 0; f < F.size(); ++f) {
        if (alive[f]) live.push_back(f);
    }
    uint64_t f = live[rng() % live.size()];
    Eigen::Vector3d here(1. / 3, 1. / 3, 1. / 3);
    SurfaceCurve curve;
    for (int s = 0; s < length; ++s) {
        const int k = int(rng() % 3);
        const uint64_t a = F[f][k], b = F[f][(k + 1) % 3];
        const auto ab = std::minmax(a, b);
        const auto& faces = on_edge[{ab.first, ab.second}];
        const uint64_t g = faces[0] == f ? faces.back() : faces[0];
        const double t = u(rng);
        Eigen::Vector3d there = Eigen::Vector3d::Zero();
        there[k] = 1 - t;
        there[(k + 1) % 3] = t;
        curve.push_back({f, {here, there}});
        // The same point in the next face: same weights on the same corners, exact zero elsewhere.
        Eigen::Vector3d next = Eigen::Vector3d::Zero();
        for (int j = 0; j < 3; ++j) {
            if (F[g][j] == a) next[j] = 1 - t;
            if (F[g][j] == b) next[j] = t;
        }
        f = g;
        here = next;
    }
    return curve;
}

/// The distance from `q` to the polyline of a curve's segments.
double distance_to(
    const Eigen::Vector3d& q,
    const SurfaceCurve& c,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F)
{
    double best = std::numeric_limits<double>::infinity();
    for (const SurfaceSegment& s : c) {
        const Eigen::Vector3d a = position(s.face, s.bary[0], V, F);
        const Eigen::Vector3d b = position(s.face, s.bary[1], V, F);
        const Eigen::Vector3d d = b - a;
        const double l2 = d.squaredNorm();
        const double t = l2 > 0 ? std::clamp((q - a).dot(d) / l2, 0., 1.) : 0.;
        best = std::min(best, (q - (a + t * d)).norm());
    }
    return best;
}

/// That `curves` went through the map in that direction as the map takes points: unbroken, end to
/// end, and every point of a curve landing on the curve's image.
void check_image(const SurfaceMap& map, const std::vector<SurfaceCurve>& curves, const bool forward)
{
    tracking::MapWarnings w;
    const auto mapped =
        forward ? map.curves_to_output(curves, &w) : map.curves_to_input(curves, &w);
    CHECK(w.total() == 0);
    const auto& V = forward ? map.output_vertices() : map.input_vertices();
    const auto& F = forward ? map.output_faces() : map.input_faces();
    const auto& alive = forward ? map.output_face_alive() : map.input_face_alive();
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e300), hi = -lo;
    for (const auto& p : V) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    const double diagonal = (hi - lo).norm();

    std::vector<SurfacePoint> samples;
    std::vector<size_t> sample_curve;
    constexpr int kPerSegment = 7;
    for (size_t c = 0; c < curves.size(); ++c) {
        for (const SurfaceSegment& s : curves[c]) {
            for (int k = 0; k <= kPerSegment; ++k) {
                const double t = double(k) / kPerSegment;
                samples.push_back({s.face, (1 - t) * s.bary[0] + t * s.bary[1]});
                sample_curve.push_back(c);
            }
        }
    }
    const auto images = forward ? map.to_output(samples) : map.to_input(samples);

    double gap = 0, off = 0;
    for (size_t c = 0; c < curves.size(); ++c) {
        REQUIRE(mapped[c].size() == 1);
        const SurfaceCurve& image = mapped[c].front();
        for (size_t i = 0; i < image.size(); ++i) {
            REQUIRE(alive[image[i].face]);
            CHECK(image[i].bary[0].minCoeff() >= -1e-9);
            CHECK(image[i].bary[1].minCoeff() >= -1e-9);
            if (i + 1 < image.size()) {
                const Eigen::Vector3d a = position(image[i].face, image[i].bary[1], V, F);
                const Eigen::Vector3d b = position(image[i + 1].face, image[i + 1].bary[0], V, F);
                gap = std::max(gap, (a - b).norm());
            }
        }
    }
    for (size_t i = 0; i < samples.size(); ++i) {
        const Eigen::Vector3d q = position(images[i].face, images[i].bary, V, F);
        off = std::max(off, distance_to(q, mapped[sample_curve[i]].front(), V, F));
    }
    INFO(
        "largest gap " << gap / diagonal << ", largest distance from a point's image "
                       << off / diagonal);
    CHECK(gap < 1e-11 * diagonal);
    CHECK(off < 1e-9 * diagonal);
}

} // namespace

TEST_CASE("segment-walk-grid", "[tracking][curves]")
{
    const auto tris = grid(6);
    SECTION("general position")
    {
        std::mt19937 rng(1);
        std::uniform_real_distribution<double> u(0.01, 5.99);
        for (int i = 0; i < 500; ++i) {
            const Eigen::Vector2d p(u(rng), u(rng)), q(u(rng), u(rng));
            const auto pieces = walk_segment(tris, locate(tris, p), locate(tris, q));
            REQUIRE(pieces);
            check_walk(tris, *pieces, p, q);
        }
    }
    SECTION("through corners and along edges")
    {
        const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> cases = {
            {{0.5, 0.25}, {5.5, 5.25}}, // parallel to the diagonals
            {{0.25, 0.25}, {5.75, 5.75}}, // along the diagonals, through corners
            {{0.5, 1}, {5.5, 1}}, // along a row of edges
            {{2, 0.5}, {2, 5.5}}, // along a column of edges
            {{0.5, 0.5}, {4.5, 2.5}}, // through corners, across cells
            {{1, 1}, {4, 1}}, // from corner to corner along edges
            {{1, 1}, {3, 3}}, // from corner to corner along the diagonals
            {{1, 1}, {3.5, 2.25}}, // from a corner across the cells
            {{0, 0}, {6, 6}}, // the boundary's corners
            {{0, 3}, {6, 3}}, // from boundary to boundary
            {{3, 3.5}, {3.5, 3.5}}, // from an edge into the next cell
        };
        for (const auto& [p, q] : cases) {
            INFO("from (" << p.x() << ", " << p.y() << ") to (" << q.x() << ", " << q.y() << ")");
            auto start = locate(tris, p), end = locate(tris, q);
            const auto pieces = walk_segment(tris, start, end);
            REQUIRE(pieces);
            check_walk(tris, *pieces, p, q);
            const auto back = walk_segment(tris, end, start);
            REQUIRE(back);
            check_walk(tris, *back, q, p);
        }
    }
    SECTION("an inverted triangle stops the walk instead of looping")
    {
        auto bad = tris;
        std::swap(bad[20][0], bad[20][1]);
        const Eigen::Vector2d p(0.3, 3.1), q(5.7, 3.4);
        const auto pieces = walk_segment(bad, locate(tris, p), locate(tris, q));
        if (pieces) check_walk(bad, *pieces, p, q);
    }
}

TEST_CASE("segment-walk-keeps-exact-zeros", "[tracking][curves]")
{
    // Where a segment goes from one triangle to the next, it is on their edge: the coordinates
    // say so exactly, and name the same point in both.
    const auto tris = grid(3);
    const Eigen::Vector2d p(0.3, 0.4), q(2.6, 1.9);
    const auto pieces = walk_segment(tris, locate(tris, p), locate(tris, q));
    REQUIRE(pieces);
    REQUIRE(pieces->size() > 2);
    for (size_t i = 0; i + 1 < pieces->size(); ++i) {
        const WalkPiece& a = (*pieces)[i];
        const WalkPiece& b = (*pieces)[i + 1];
        CHECK(a.bary[1].minCoeff() == 0);
        CHECK(b.bary[0].minCoeff() == 0);
        const auto carried = tracking::carry(tris[a.tri], a.bary[1], tris[b.tri]);
        REQUIRE(carried);
        CHECK(*carried == b.bary[0]);
    }
}

TEST_CASE("surface-curves-identity", "[tracking][curves]")
{
    // A map of no edits leaves a curve as it is.
    history::SurfaceHistory h;
    h.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    h.faces = {{0, 1, 2}, {0, 2, 3}};
    h.face_alive = {1, 1};
    h.vertex_alive = {1, 1, 1, 1};
    const SurfaceMap map(h);
    const SurfaceCurve c = {
        {0, {Eigen::Vector3d(0.5, 0.3, 0.2), Eigen::Vector3d(0.5, 0, 0.5)}},
        {1, {Eigen::Vector3d(0.5, 0.5, 0), Eigen::Vector3d(0.1, 0.1, 0.8)}}};
    tracking::MapWarnings w;
    const auto out = map.curves_to_output({c}, &w);
    CHECK(w.total() == 0);
    REQUIRE(out.size() == 1);
    REQUIRE(out[0].size() == 1);
    REQUIRE(out[0][0].size() == 2);
    CHECK(out[0][0][1].bary[1] == c[1].bary[1]);
}

TEST_CASE("surface-curves-face-with-no-image", "[tracking][curves]")
{
    // A curve over a face that does not exist breaks into the parts around it, counted.
    history::SurfaceHistory h;
    h.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    h.faces = {{0, 1, 2}, {0, 2, 3}};
    h.face_alive = {1, 1};
    h.vertex_alive = {1, 1, 1, 1};
    const SurfaceMap map(h);
    const Eigen::Vector3d a(0.5, 0.3, 0.2), b(0.2, 0.3, 0.5);
    tracking::MapWarnings w;
    const auto out = map.curves_to_output({{{0, {a, b}}, {7, {a, b}}, {1, {a, b}}}}, &w);
    CHECK(w.broken_segments == 1);
    REQUIRE(out[0].size() == 2);
    CHECK(out[0][0][0].face == 0);
    CHECK(out[0][1][0].face == 1);
}

TEST_CASE("surface-curves-through-remeshing", "[tracking][curves]" + release_only)
{
    // A closed manifold, remeshed: every local map a bijection, so a curve's image is exact and
    // agrees with the point map all along it, in both directions.
    const auto h = remesh("sphere.obj", 0.6, 2);
    const SurfaceMap map(h);
    REQUIRE(map.build_warnings().total() == 0);
    std::mt19937 rng(7);

    std::vector<SurfaceCurve> on_input, on_output;
    for (int c = 0; c < 20; ++c) {
        on_input.push_back(random_curve(map.input_faces(), map.input_face_alive(), 12, rng));
        on_output.push_back(random_curve(map.output_faces(), map.output_face_alive(), 12, rng));
    }
    check_image(map, on_input, true);
    check_image(map, on_output, false);
}

TEST_CASE("surface-curves-round-trip", "[tracking][curves]" + release_only)
{
    // Through the map and back, a curve lies where it was: more segments, the same curve.
    const auto h = remesh("sphere.obj", 0.6, 2);
    const SurfaceMap map(h);
    std::mt19937 rng(3);
    std::vector<SurfaceCurve> curves;
    for (int c = 0; c < 10; ++c) {
        curves.push_back(random_curve(map.input_faces(), map.input_face_alive(), 8, rng));
    }
    tracking::MapWarnings w;
    std::vector<SurfaceCurve> out;
    for (const auto& parts : map.curves_to_output(curves, &w)) {
        REQUIRE(parts.size() == 1);
        out.push_back(parts.front());
    }
    const auto back = map.curves_to_input(out, &w);
    CHECK(w.total() == 0);
    const auto& V = map.input_vertices();
    const auto& F = map.input_faces();
    double off = 0;
    for (size_t c = 0; c < curves.size(); ++c) {
        REQUIRE(back[c].size() == 1);
        for (const SurfaceSegment& s : curves[c]) {
            for (const auto& b : s.bary) {
                off = std::max(off, distance_to(position(s.face, b, V, F), back[c][0], V, F));
            }
        }
    }
    CHECK(off < 1e-9);
}

TEST_CASE("surface-curves-across-a-feature-curve", "[tracking][curves]" + release_only)
{
    // A feature curve cuts patches into sides, and a collapse along it splits a triangle virtually:
    // the face bends where the map does, and a curve going from side to side stays connected.
    const auto h = remesh_with_curve(0.55, 0.6, 2);
    const SurfaceMap map(h);
    REQUIRE(map.build_warnings().total() == 0);
    REQUIRE(map.curve_collapses() > 0);
    std::mt19937 rng(11);
    std::vector<SurfaceCurve> on_input, on_output;
    for (int c = 0; c < 40; ++c) {
        on_input.push_back(random_curve(map.input_faces(), map.input_face_alive(), 20, rng));
        on_output.push_back(random_curve(map.output_faces(), map.output_face_alive(), 20, rng));
    }
    check_image(map, on_input, true);
    check_image(map, on_output, false);
}

TEST_CASE("surface-map-agrees-across-a-feature-curve", "[tracking][surfaces]" + release_only)
{
    // A point on a feature edge has a name on each side, and both must map to one place. A vertex
    // on a curve that barely moved used to part them: rounding showed the move to one side alone,
    // which clamped it to a hundredth of the edge.
    const auto h = remesh_with_curve(0.55, 0.6, 2);
    const SurfaceMap map(h);
    const auto& F = map.input_faces();
    std::map<std::pair<uint64_t, uint64_t>, std::vector<uint64_t>> on_edge;
    for (uint64_t f = 0; f < F.size(); ++f) {
        if (!map.input_face_alive()[f]) continue;
        for (int k = 0; k < 3; ++k) {
            const auto ab = std::minmax(F[f][k], F[f][(k + 1) % 3]);
            on_edge[{ab.first, ab.second}].push_back(f);
        }
    }
    std::vector<SurfacePoint> one, other;
    for (const auto& e : h.features) {
        const auto& faces = on_edge[{std::min(e[0], e[1]), std::max(e[0], e[1])}];
        REQUIRE(faces.size() == 2);
        for (const double t : {0.13, 0.37, 0.61, 0.87}) {
            const auto point = [&](const uint64_t f) {
                Eigen::Vector3d l = Eigen::Vector3d::Zero();
                for (int j = 0; j < 3; ++j) {
                    if (F[f][j] == e[0]) l[j] = 1 - t;
                    if (F[f][j] == e[1]) l[j] = t;
                }
                return SurfacePoint{f, l};
            };
            one.push_back(point(faces[0]));
            other.push_back(point(faces[1]));
        }
    }
    const auto a = map.to_output(one), b = map.to_output(other);
    double gap = 0;
    const auto& V = map.output_vertices();
    const auto& G = map.output_faces();
    for (size_t i = 0; i < a.size(); ++i) {
        gap = std::max(
            gap,
            (position(a[i].face, a[i].bary, V, G) - position(b[i].face, b[i].bary, V, G)).norm());
    }
    CHECK(gap < 1e-11);
}
