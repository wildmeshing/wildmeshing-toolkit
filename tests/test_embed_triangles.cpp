#include <wmtk/utils/EmbedTriangles.hpp>

#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <map>
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

TEST_CASE("embed-triangles-orientation-per-input", "[embed_triangles][orientation]")
{
    // EmbedTrianglesProvenance::face_input_orientation: each input's own count. A on
    // [0,1]x[0,1] (input 0, +z) and a copy of it (input 1, +z or -z) are one coplanar group,
    // which the one count takes once: 1 where the copy faces alike, 0 where it faces the other
    // way. Input by input each counts: +1 and +1, or +1 and -1. Beside them B (input 2, -z),
    // edge-adjacent to A in the same group, counts for input 2 alone. The arrangement does not
    // cut a group along the edge between two of its triangles, so a facet may lie across x = 1:
    // it is then on every input whose triangles it overlaps (face_triangles), each with its own
    // count, and the one count is that at its first test point (facet_orientations()).
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // TetMesh::m_local_faces
    for (const bool copy_up : {true, false}) {
        CAPTURE(copy_up);
        Sheets s = two_sheets(false);
        s.F.insert(s.F.end(), {0, 1, 2, 0, 2, 3});
        if (!copy_up) {
            std::swap(s.F[13], s.F[14]);
            std::swap(s.F[16], s.F[17]);
        }
        const std::vector<int> inputs = {0, 0, 2, 2, 1, 1};
        std::vector<double> TV;
        std::vector<uint32_t> TT;
        background(TV, TT);
        Embedded e;
        e.prov.want_input_orientation = true;
        e.prov.want_face_triangles = true;
        utils::EmbedTrianglesOptions opts;
        opts.triangle_input = &inputs;
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
        REQUIRE(e.prov.face_input_orientation.size() == e.faces.size());
        std::map<std::array<size_t, 3>, size_t> facet_of;
        for (size_t f = 0; f < e.faces.size(); ++f) {
            std::array<size_t, 3> key = e.faces[f];
            std::sort(key.begin(), key.end());
            facet_of[key] = f;
        }
        const std::array<int, 3> facing = {{1, copy_up ? 1 : -1, -1}}; // per input, +z or -z
        size_t n_coincident = 0, n_b = 0, wrong = 0;
        for (size_t t = 0; t < e.tets.size(); ++t) {
            for (int k = 0; k < 4; ++k) {
                if (!e.tet_face_on_input[4 * t + k]) continue;
                std::array<size_t, 3> f;
                for (int j = 0; j < 3; ++j) f[j] = e.tets[t][local_faces[k][j]];
                std::sort(f.begin(), f.end());
                const size_t facet = facet_of.at(f);
                const Vector3d a = to_double(e.v[f[0]]), b = to_double(e.v[f[1]]),
                               c = to_double(e.v[f[2]]);
                const int up = (b - a).cross(c - a)[2] > 0 ? 1 : -1; // +z, against ascending
                std::vector<int> on;
                for (const uint32_t tri : e.prov.face_triangles[facet]) on.push_back(inputs[tri]);
                std::sort(on.begin(), on.end());
                on.erase(std::unique(on.begin(), on.end()), on.end());
                InputOrientations want;
                for (const int in : on) want.emplace_back(in, facing[in] * up);
                wrong += e.prov.face_input_orientation[facet] != want;
                const bool on_a = on.front() < 2, on_b = on.back() == 2;
                if (on_a != on_b) { // on one side of x = 1: the one count as before
                    wrong += e.orientation[4 * t + k] != (on_a ? (copy_up ? up : 0) : -up);
                }
                n_coincident += on.size() >= 2 && on[0] == 0 && on[1] == 1;
                n_b += on_b;
            }
        }
        CHECK(n_coincident > 0);
        CHECK(n_b > 0);
        CHECK(wrong == 0);
    }
}

TEST_CASE("surface-tag-orientation-carried", "[orientation]")
{
    // What the operations do with the orientation (SurfaceTagAttributes): a count along one
    // vertex order is the same count along the corresponding order of the simplex it moves to
    // (set_orientation_from), a merge adds (add_orientation_from), and alike means equal along
    // corresponding orders (same_orientation). Every input's count with WMTK_PER_INPUT_ORIENTATION.
    using A3 = std::array<size_t, 3>;
    SurfaceTagAttributes f;
    f.set_input_set(0b11);
    f.set_orientation_along(InputSet(0b01), A3{{5, 3, 7}}, 1);
    CHECK(f.orientation_along(A3{{5, 3, 7}}) == 1);
    CHECK(f.orientation() == -1); // (5, 3, 7) is an odd permutation of (3, 5, 7)
    CHECK(f.oriented());
    SurfaceTagAttributes half;
    half.set_orientation_from(f, A3{{5, 3, 7}}, A3{{5, 9, 7}}); // 3 replaced by 9 in place
    CHECK(half.orientation_along(A3{{5, 9, 7}}) == 1);
    CHECK(half.same_orientation(A3{{5, 9, 7}}, f, A3{{5, 3, 7}}));
    CHECK_FALSE(half.same_orientation(A3{{9, 5, 7}}, f, A3{{5, 3, 7}}));
    SurfaceTagAttributes merged;
    merged.set_orientation_from(f, A3{{5, 3, 7}}, A3{{5, 3, 8}});
    merged.add_orientation_from(half, A3{{5, 9, 7}}, A3{{5, 3, 8}});
    CHECK(merged.orientation_along(A3{{5, 3, 8}}) == 2);
    merged.reset();
    CHECK(merged.orientation() == 0);
    CHECK_FALSE(merged.oriented());

#ifdef WMTK_PER_INPUT_ORIENTATION
    // One count per input, 4 bits each, and the total their sum: two inputs facing opposite
    // ways on one face sum to 0, each keeping its own.
    STATIC_REQUIRE(sizeof(SurfaceTagAttributes) == 43);
    f.set_orientation_along(InputSet(0b10), A3{{3, 5, 7}}, 1);
    CHECK(f.input_orientation(0) == -1);
    CHECK(f.input_orientation(1) == 1);
    CHECK(f.orientation() == 0);
    CHECK(f.oriented());
    CHECK(f.orientation_of(0b01) == -1);
    CHECK(f.orientation_of(0b10) == 1);
    // Along (3, 5, 7) taken as along (5, 3, 7), the other orientation of the same vertex set:
    // every input's count turned over.
    half.set_orientation_from(f, A3{{3, 5, 7}}, A3{{5, 3, 7}});
    CHECK(half.input_orientation(0) == 1);
    CHECK(half.input_orientation(1) == -1);
    CHECK(half.same_orientation(A3{{5, 3, 7}}, f, A3{{3, 5, 7}}));
    half.set_input_orientation(1, 0); // input 0 still alike, input 1 not
    CHECK_FALSE(half.same_orientation(A3{{5, 3, 7}}, f, A3{{3, 5, 7}}));
    // Clamped to -8..7; the total is exact while every count is in range.
    SurfaceTagAttributes c;
    c.set_orientation(0b111, 7);
    CHECK(c.orientation() == 21);
    c.set_orientation(0b001, 20);
    c.set_orientation(0b010, -20);
    CHECK(c.input_orientation(0) == 7);
    CHECK(c.input_orientation(1) == -8);
    c.set_input_orientations({{2, -3}, {63, 5}});
    CHECK(c.input_orientation(0) == 0);
    CHECK(c.input_orientation(2) == -3);
    CHECK(c.input_orientation(63) == 5);
    CHECK(c.orientation() == 2);
#else
    // One count for all the inputs: set for one input, it is every input's.
    STATIC_REQUIRE(sizeof(SurfaceTagAttributes) == 11);
    CHECK(f.orientation_of(0b10) == -1);
    CHECK(f.orientation_of(0b100) == 0); // not on input 2
    SurfaceTagAttributes c;
    c.set_orientation(0b1, 20);
    CHECK(c.orientation() == 15);
#endif
}
