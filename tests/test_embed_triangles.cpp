#include <wmtk/utils/EmbedTriangles.hpp>

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

using namespace wmtk;

namespace {

// Two squares at z = 0.25 side by side, A on [0,1]x[0,1] (triangles 0, 1) and B on
// [1,1.5]x[0,1] (triangles 2, 3), sharing the edge x = 1. Exactly coplanar and edge-adjacent, so
// the arrangement puts all four triangles in ONE coplanar group: the case where the group alone
// cannot say which sheet a facet lies on. A faces +z; B faces +z or, with `b_up` false, -z.
struct Sheets
{
    std::vector<double> V;
    std::vector<uint32_t> F;
};

Sheets two_sheets(const bool b_up = true)
{
    Sheets s;
    s.V = {0, 0, 0.25, 1, 0, 0.25, 1, 1, 0.25, 0, 1, 0.25, 1.5, 0, 0.25, 1.5, 1, 0.25};
    s.F = {0, 1, 2, 0, 2, 3};
    if (b_up) {
        s.F.insert(s.F.end(), {1, 4, 5, 1, 5, 2});
    } else {
        s.F.insert(s.F.end(), {1, 5, 4, 1, 2, 5});
    }
    return s;
}

// The box [-1,3]x[-1,2]x[-1,1] as six tets around its main diagonal, positively oriented.
void background(std::vector<double>& V, std::vector<uint32_t>& T)
{
    V.clear();
    for (int c = 0; c < 8; ++c) {
        V.insert(V.end(), {c & 1 ? 3. : -1., c & 2 ? 2. : -1., c & 4 ? 1. : -1.});
    }
    const auto p = [&](uint32_t i) { return Vector3d(V[3 * i], V[3 * i + 1], V[3 * i + 2]); };
    std::array<int, 3> axes = {{0, 1, 2}};
    T.clear();
    do {
        const uint32_t a = 0, b = 1u << axes[0], c = b | (1u << axes[1]), d = 7;
        const double vol = (p(b) - p(a)).cross(p(c) - p(a)).dot(p(d) - p(a));
        if (vol > 0) {
            T.insert(T.end(), {a, b, c, d});
        } else {
            T.insert(T.end(), {a, b, d, c});
        }
    } while (std::next_permutation(axes.begin(), axes.end()));
}

struct Embedded
{
    std::vector<Vector3r> v;
    std::vector<std::array<size_t, 3>> faces;
    std::vector<bool> on_input, v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input;
    std::vector<int8_t> orientation;
    utils::EmbedTrianglesProvenance prov;
};

Embedded embed(
    const Sheets& s,
    const bool want_face_triangles,
    const std::vector<int>* triangle_input = nullptr)
{
    std::vector<double> TV;
    std::vector<uint32_t> TT;
    background(TV, TT);
    Embedded e;
    e.prov.want_face_triangles = want_face_triangles;
    utils::EmbedTrianglesOptions opts;
    opts.triangle_input = triangle_input;
    utils::embed_triangles_in_tets(
        s.V,
        s.F,
        TV,
        TT,
        e.v,
        e.faces,
        e.on_input,
        e.v_on_input,
        e.tets,
        e.tet_face_on_input,
        opts,
        &e.prov,
        &e.orientation);
    return e;
}

} // namespace

TEST_CASE("embed-triangles-face-triangles", "[embed_triangles]")
{
    const Embedded e = embed(two_sheets(), true);

    // The case under test: both sheets in one coplanar group.
    REQUIRE(e.prov.triangle_group.size() == 4);
    for (size_t t = 1; t < 4; ++t) REQUIRE(e.prov.triangle_group[t] == e.prov.triangle_group[0]);

    // Every facet on the input lists the triangles it overlaps; a facet on one side of x = 1
    // lists none of the other sheet's.
    REQUIRE(e.prov.face_triangles.size() == e.faces.size());
    size_t n_a = 0, n_b = 0;
    for (size_t f = 0; f < e.faces.size(); ++f) {
        const auto& ts = e.prov.face_triangles[f];
        if (!e.on_input[f]) {
            CHECK(ts.empty());
            continue;
        }
        REQUIRE(!ts.empty());
        double x_min = std::numeric_limits<double>::max(), x_max = -x_min;
        for (const size_t v : e.faces[f]) {
            x_min = std::min(x_min, to_double(e.v[v])[0]);
            x_max = std::max(x_max, to_double(e.v[v])[0]);
        }
        const bool on_a = std::any_of(ts.begin(), ts.end(), [](uint32_t t) { return t < 2; });
        const bool on_b = std::any_of(ts.begin(), ts.end(), [](uint32_t t) { return t >= 2; });
        if (x_max <= 1) CHECK(!on_b);
        if (x_min >= 1) CHECK(!on_a);
        n_a += on_a;
        n_b += on_b;
    }
    CHECK(n_a > 0);
    CHECK(n_b > 0);

    // Off by default: nothing is built unless asked.
    const Embedded off = embed(two_sheets(), false);
    CHECK(off.prov.face_triangles.empty());
}

TEST_CASE("embed-triangles-orientation-keeps-to-each-input", "[embed_triangles]")
{
    // B faces -z next to A facing +z, sharing an edge. Told which input each triangle is, the
    // repair keeps to one input and each tet face on the input carries its own sheet's
    // orientation. As one input, B is the patch's minority (half A's area) and is reversed to
    // agree with A -- the single-input repair, unchanged.
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // TetMesh::m_local_faces
    const std::vector<int> two_inputs = {0, 0, 1, 1};
    for (const bool b_up : {true, false}) {
        for (const bool told : {true, false}) {
            CAPTURE(b_up, told);
            const Embedded e = embed(two_sheets(b_up), false, told ? &two_inputs : nullptr);
            size_t n_a = 0, n_b = 0, wrong_a = 0, wrong_b = 0;
            for (size_t t = 0; t < e.tets.size(); ++t) {
                for (int k = 0; k < 4; ++k) {
                    if (!e.tet_face_on_input[4 * t + k]) continue;
                    std::array<size_t, 3> f;
                    for (int j = 0; j < 3; ++j) f[j] = e.tets[t][local_faces[k][j]];
                    std::sort(f.begin(), f.end());
                    const Vector3d a = to_double(e.v[f[0]]), b = to_double(e.v[f[1]]),
                                   c = to_double(e.v[f[2]]);
                    const double nz = (b - a).cross(c - a)[2];
                    const int o = e.orientation[4 * t + k];
                    const bool side_a = (a[0] + b[0] + c[0]) / 3 < 1;
                    const int want = side_a || b_up || !told ? 1 : -1; // the normal is want * +z
                    const bool right = o * nz * want > 0;
                    if (side_a) {
                        ++n_a;
                        wrong_a += !right;
                    } else {
                        ++n_b;
                        wrong_b += !right;
                    }
                }
            }
            CHECK(n_a > 0);
            CHECK(n_b > 0);
            CHECK(wrong_a == 0);
            CHECK(wrong_b == 0);
        }
    }
}
