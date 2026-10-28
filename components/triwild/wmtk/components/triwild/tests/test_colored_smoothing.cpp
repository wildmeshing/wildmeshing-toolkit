#include <wmtk/components/triwild/TriWildMesh.h>
#include <wmtk/utils/EmbedSegments.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

// A jittered n x n grid of squares, each cut into two triangles. Every interior vertex has room
// to move, so a smoothing pass does real work on most of them; the boundary vertices are marked
// as lying on the bounding box, which keeps them fixed.
void load_jittered_grid(TriWildMesh& mesh, const size_t n, const unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const auto vid = [n](size_t i, size_t j) { return j * n + i; };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j + 1 < n; ++j) {
        for (size_t i = 0; i + 1 < n; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
            tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    mesh.init(n * n, tris);
    mesh.m_vertex_attribute.resize(n * n);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < n; ++i) {
            auto& va = mesh.m_vertex_attribute[vid(i, j)];
            if (i == 0) va.on_bbox_faces.push_back(0);
            if (i == n - 1) va.on_bbox_faces.push_back(1);
            if (j == 0) va.on_bbox_faces.push_back(2);
            if (j == n - 1) va.on_bbox_faces.push_back(3);
            va.m_posf = Vector2d(double(i), double(j));
            if (va.on_bbox_faces.empty()) va.m_posf += Vector2d(jitter(rng), jitter(rng));
            va.set_pos_to_posf();
            va.m_is_rounded = true;
            va.m_is_on_surface = false;
            va.m_sizing_scalar = 1.0;
        }
    }
    for (size_t f = 0; f < tris.size(); ++f) {
        mesh.m_face_attribute[f].m_quality = mesh.get_quality(mesh.oriented_tri_vids(f));
    }
}

/// Compares every vertex (double and exact position, rounded flag) and every cached quality.
void check_same_mesh(const TriWildMesh& a, const TriWildMesh& b)
{
    REQUIRE(a.vert_capacity() == b.vert_capacity());
    REQUIRE(a.tri_capacity() == b.tri_capacity());
    size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
    for (size_t v = 0; v < a.vert_capacity(); ++v) {
        const auto& x = a.m_vertex_attribute[v];
        const auto& y = b.m_vertex_attribute[v];
        if (x.m_posf != y.m_posf) ++pos_diff;
        if (!(x.pos() == y.pos()) || x.m_is_rounded != y.m_is_rounded) ++exact_diff;
    }
    for (size_t f = 0; f < a.tri_capacity(); ++f) {
        if (a.m_face_attribute[f].m_quality != b.m_face_attribute[f].m_quality) ++quality_diff;
    }
    CHECK(pos_diff == 0);
    CHECK(exact_diff == 0);
    CHECK(quality_diff == 0);
}

/// Takes every k-th rounded interior vertex off the double grid, by a third of 1e-9 in x: its
/// exact position is then not a double, so it is unrounded, and smoothing has to round it first.
/// Deterministic, so every mesh it is applied to gets the same vertices. Returns how many.
size_t unround_some(TriWildMesh& mesh, const size_t k)
{
    const Rational third = Rational(1e-9) / Rational(3.0);
    size_t n = 0, i = 0;
    for (const auto& t : mesh.get_vertices()) {
        auto& a = mesh.m_vertex_attribute[t.vid(mesh)];
        if (a.m_is_on_surface || !a.on_bbox_faces.empty() || !a.m_is_rounded) continue;
        if (i++ % k != 0) continue;
        Vector2r p = a.pos();
        p[0] = p[0] + third;
        a.set_pos(p);
        a.m_posf = to_double(p);
        a.m_is_rounded = false;
        ++n;
    }
    return n;
}

size_t count_unrounded(const TriWildMesh& mesh)
{
    size_t n = 0;
    for (const auto& t : mesh.get_vertices())
        n += !mesh.m_vertex_attribute[t.vid(mesh)].m_is_rounded;
    return n;
}

/// Smoothing that reads a neighbour through NON-const access and then rejects the move: the
/// neighbour is recorded and written back, the bug the colored pass checks for.
class NeighbourReadingMesh : public TriWildMesh
{
public:
    using TriWildMesh::TriWildMesh;
    bool smooth_after(const Tuple& t) override
    {
        const size_t vid = t.vid(*this);
        for (const size_t u : get_one_ring_vids_for_vertex_duplicate(vid)) {
            if (u != vid) {
                (void)m_vertex_attribute[u].m_posf;
                break;
            }
        }
        return false;
    }
};

} // namespace

TEST_CASE("colored smoothing does not depend on the number of threads (2D)", "[triwild][smoothing]")
{
    // The 2D twin of the tetwild test: no two vertices of a color class interact, so every
    // thread count must give the same mesh, bit for bit.
    constexpr size_t n = 48;
    std::vector<Vector2d> initial;

    struct Run
    {
        Parameters params;
        std::unique_ptr<TriWildMesh> mesh;
    };
    std::vector<Run> runs;
    // Each mesh holds a reference to its Run's params: the vector must not reallocate.
    runs.reserve(4);
    for (const int threads : {1, 3, 8, 16}) {
        runs.emplace_back();
        Run& r = runs.back();
        r.params.init(Vector2d(-1, -1), Vector2d(double(n), double(n)));
        r.mesh = std::make_unique<TriWildMesh>(r.params, r.params.eps, threads);
        load_jittered_grid(*r.mesh, n, 11);
        if (initial.empty()) {
            for (size_t v = 0; v < n * n; ++v) {
                initial.push_back(r.mesh->m_vertex_attribute[v].m_posf);
            }
        }
        REQUIRE(r.mesh->use_colored_smoothing());
        r.mesh->smooth_all_vertices(2);
    }

    const TriWildMesh& ref = *runs.front().mesh;
    size_t moved = 0;
    for (size_t v = 0; v < n * n; ++v) {
        if (ref.m_vertex_attribute[v].m_posf != initial[v]) ++moved;
    }
    CHECK(moved > n * n / 4); // the passes really did something

    for (size_t r = 1; r < runs.size(); ++r) {
        const TriWildMesh& other = *runs[r].mesh;
        size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
        for (size_t v = 0; v < n * n; ++v) {
            const auto& a = ref.m_vertex_attribute[v];
            const auto& b = other.m_vertex_attribute[v];
            if (a.m_posf != b.m_posf) ++pos_diff;
            if (!(a.pos() == b.pos()) || a.m_is_rounded != b.m_is_rounded) ++exact_diff;
        }
        for (size_t f = 0; f < 2 * (n - 1) * (n - 1); ++f) {
            if (ref.m_face_attribute[f].m_quality != other.m_face_attribute[f].m_quality) {
                ++quality_diff;
            }
        }
        CHECK(pos_diff == 0);
        CHECK(exact_diff == 0);
        CHECK(quality_diff == 0);
    }
}

TEST_CASE(
    "colored smoothing with a curve does not depend on the number of threads (2D)",
    "[triwild][smoothing]")
{
    // The same property where smoothing does the most: a slanted closed curve, cut by a chord,
    // embedded and refined by one serial optimization round (serial, so identical for every
    // run), then smoothed on 1, 4 and 16 threads. Its vertices are smoothed against the envelope,
    // and some interior vertices are made unrounded so that smoothing has to round them first.
    constexpr int n_curve = 48;
    MatrixXd V(n_curve + 2, 2);
    MatrixXi E(n_curve + 1, 2);
    for (int i = 0; i < n_curve; ++i) {
        const double a = 2 * 3.14159265358979323846 * i / n_curve; // M_PI is not portable
        V.row(i) << 0.5 + 0.43 * std::cos(a + 0.3), 0.5 + 0.29 * std::sin(a) + 0.07 * std::cos(a);
        E.row(i) << i, (i + 1) % n_curve;
    }
    V.row(n_curve) << 0.11, 0.37;
    V.row(n_curve + 1) << 0.93, 0.61;
    E.row(n_curve) << n_curve, n_curve + 1;

    std::vector<std::unique_ptr<Parameters>> params;
    std::vector<std::unique_ptr<TriWildMesh>> meshes;
    for (const int threads : {1, 4, 16}) {
        params.push_back(std::make_unique<Parameters>());
        params.back()->init(V.colwise().minCoeff(), V.colwise().maxCoeff());
        MatrixXd Vo;
        std::vector<Vector2r> Vr;
        MatrixXi F, Eo;
        std::vector<int> orientation;
        utils::embed_segments(V, E, Vo, Vr, F, Eo, nullptr, &orientation);
        meshes.push_back(std::make_unique<TriWildMesh>(*params.back(), params.back()->eps, 0));
        TriWildMesh& mesh = *meshes.back();
        mesh.init_mesh(Vo, Vr, F, Eo, {"input"}, V, E, &orientation);
        mesh.mesh_improvement(1);

        mesh.NUM_THREADS = threads;
        REQUIRE(mesh.use_colored_smoothing());
        size_t surface = 0;
        for (const auto& t : mesh.get_vertices()) {
            surface += mesh.m_vertex_attribute[t.vid(mesh)].m_is_on_surface;
        }
        REQUIRE(surface > 0);
        REQUIRE(unround_some(mesh, 5) > 0);
        const size_t unrounded_before = count_unrounded(mesh);
        mesh.smooth_all_vertices(2);
        CHECK(count_unrounded(mesh) < unrounded_before); // smoothing rounded some of them
    }
    for (size_t r = 1; r < meshes.size(); ++r) check_same_mesh(*meshes.front(), *meshes[r]);
}

TEST_CASE("colored smoothing rejects a hook that writes a neighbour (2D)", "[triwild][smoothing]")
{
    // The 2D twin of the tetwild test: the colored pass checks every smooth and throws on the
    // first non-const access outside the vertex's star, on any number of threads.
    constexpr size_t n = 8;
    for (const int threads : {1, 4}) {
        CAPTURE(threads);
        Parameters params;
        params.init(Vector2d(-1, -1), Vector2d(double(n), double(n)));
        NeighbourReadingMesh mesh(params, params.eps, threads);
        load_jittered_grid(mesh, n, 11);
        REQUIRE(mesh.use_colored_smoothing());
        try {
            mesh.smooth_all_vertices(1);
            FAIL("the colored pass accepted a hook that writes a neighbour");
        } catch (const std::runtime_error& e) {
            CHECK(std::string(e.what()).find("outside its star") != std::string::npos);
        }
    }
    // The locked pass does not check: its ring locks keep the neighbour to one thread.
    Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(double(n), double(n)));
    params.colored_smoothing = false;
    NeighbourReadingMesh mesh(params, params.eps, 4);
    load_jittered_grid(mesh, n, 11);
    REQUIRE_FALSE(mesh.use_colored_smoothing());
    REQUIRE_NOTHROW(mesh.smooth_all_vertices(1));
}
