#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <wmtk/components/tetwild/TetWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

// A jittered n x n x n grid, each cube cut into the six tets around its main diagonal. Every
// interior vertex has room to move, so a smoothing pass does real work on most of them; the
// boundary vertices are marked as lying on the bounding box, which keeps them fixed.
struct JitteredGrid
{
    std::vector<Vector3d> points;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::vector<int>> on_bbox;
};

JitteredGrid make_jittered_grid(const int n, const unsigned seed)
{
    JitteredGrid g;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const auto id = [n](int i, int j, int k) { return size_t((i * n + j) * n + k); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < n; ++k) {
                std::vector<int> faces;
                if (i == 0) faces.push_back(0);
                if (i == n - 1) faces.push_back(1);
                if (j == 0) faces.push_back(2);
                if (j == n - 1) faces.push_back(3);
                if (k == 0) faces.push_back(4);
                if (k == n - 1) faces.push_back(5);
                Vector3d p(i, j, k);
                if (faces.empty()) p += Vector3d(jitter(rng), jitter(rng), jitter(rng));
                g.points.push_back(p);
                g.on_bbox.push_back(faces);
            }
        }
    }
    static constexpr std::array<std::array<int, 4>, 6> kKuhn = {
        {{{0, 1, 3, 7}},
         {{0, 1, 5, 7}},
         {{0, 2, 3, 7}},
         {{0, 2, 6, 7}},
         {{0, 4, 5, 7}},
         {{0, 4, 6, 7}}}};
    for (int i = 0; i + 1 < n; ++i) {
        for (int j = 0; j + 1 < n; ++j) {
            for (int k = 0; k + 1 < n; ++k) {
                const std::array<size_t, 8> c = {
                    {id(i, j, k),
                     id(i + 1, j, k),
                     id(i, j + 1, k),
                     id(i + 1, j + 1, k),
                     id(i, j, k + 1),
                     id(i + 1, j, k + 1),
                     id(i, j + 1, k + 1),
                     id(i + 1, j + 1, k + 1)}};
                for (const auto& t : kKuhn) {
                    std::array<size_t, 4> tet = {{c[t[0]], c[t[1]], c[t[2]], c[t[3]]}};
                    const Vector3d& p0 = g.points[tet[0]];
                    const double vol = (g.points[tet[1]] - p0)
                                           .cross(g.points[tet[2]] - p0)
                                           .dot(g.points[tet[3]] - p0);
                    if (vol < 0) std::swap(tet[2], tet[3]); // wmtk orientation: positive volume
                    g.tets.push_back(tet);
                }
            }
        }
    }
    return g;
}

void load(TetWildMesh& mesh, const JitteredGrid& g)
{
    mesh.init(g.points.size(), g.tets);
    mesh.m_vertex_attribute.resize(g.points.size());
    mesh.m_face_attribute.resize(4 * g.tets.size());
    mesh.m_tet_attribute.resize(g.tets.size());
    for (size_t v = 0; v < g.points.size(); ++v) {
        auto& a = mesh.m_vertex_attribute[v];
        a.m_posf = g.points[v];
        a.set_pos_to_posf();
        a.m_is_rounded = true;
        a.on_bbox_faces = g.on_bbox[v];
    }
    for (size_t t = 0; t < g.tets.size(); ++t) {
        mesh.m_tet_attribute[t].m_quality = mesh.get_quality(mesh.tuple_from_tet(t));
    }
}

/// Compares every vertex (double and exact position, rounded flag) and every cached tet quality.
void check_same_mesh(const TetWildMesh& a, const TetWildMesh& b)
{
    REQUIRE(a.vert_capacity() == b.vert_capacity());
    REQUIRE(a.tet_capacity() == b.tet_capacity());
    size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
    for (size_t v = 0; v < a.vert_capacity(); ++v) {
        const auto& x = a.m_vertex_attribute[v];
        const auto& y = b.m_vertex_attribute[v];
        if (x.m_posf != y.m_posf) ++pos_diff;
        if (!(x.pos() == y.pos()) || x.m_is_rounded != y.m_is_rounded) ++exact_diff;
    }
    for (size_t t = 0; t < a.tet_capacity(); ++t) {
        if (a.m_tet_attribute[t].m_quality != b.m_tet_attribute[t].m_quality) ++quality_diff;
    }
    CHECK(pos_diff == 0);
    CHECK(exact_diff == 0);
    CHECK(quality_diff == 0);
}

/// Takes every k-th rounded interior vertex off the double grid, by a third of 1e-9 in x: its
/// exact position is then not a double, so it is unrounded, and smoothing has to round it first.
/// Deterministic, so every mesh it is applied to gets the same vertices. Returns how many.
size_t unround_some(TetWildMesh& mesh, const size_t k)
{
    const Rational third = Rational(1e-9) / Rational(3.0);
    size_t n = 0, i = 0;
    for (const auto& t : mesh.get_vertices()) {
        auto& a = mesh.m_vertex_attribute[t.vid(mesh)];
        if (a.m_is_on_surface || !a.on_bbox_faces.empty() || !a.m_is_rounded) continue;
        if (i++ % k != 0) continue;
        Vector3r p = a.pos();
        p[0] = p[0] + third;
        a.set_pos(p);
        a.m_posf = to_double(p);
        a.m_is_rounded = false;
        ++n;
    }
    return n;
}

size_t count_unrounded(const TetWildMesh& mesh)
{
    size_t n = 0;
    for (const auto& t : mesh.get_vertices())
        n += !mesh.m_vertex_attribute[t.vid(mesh)].m_is_rounded;
    return n;
}

/// Smoothing that reads a neighbour through NON-const access and then rejects the move: the
/// neighbour is recorded and written back, the bug the colored pass checks for.
class NeighbourReadingMesh : public TetWildMesh
{
public:
    using TetWildMesh::TetWildMesh;
    bool smooth_after(const Tuple& t) override
    {
        const size_t vid = t.vid(*this);
        for (const size_t u : get_one_ring_vids_for_vertex(vid)) {
            if (u != vid) {
                (void)m_vertex_attribute[u].m_posf;
                break;
            }
        }
        return false;
    }
};

} // namespace

TEST_CASE("colored smoothing does not depend on the number of threads", "[tetwild][smoothing]")
{
    // The point of smoothing by color class: no two vertices of a class interact, so which thread
    // smooths which vertex cannot matter, and every thread count must give the same mesh, bit for
    // bit -- positions, exact positions and the cached cell qualities.
    const JitteredGrid grid = make_jittered_grid(16, 7);

    struct Run
    {
        Parameters params;
        std::unique_ptr<TetWildMesh> mesh;
    };
    std::vector<Run> runs;
    // Each mesh holds a reference to its Run's params: the vector must not reallocate.
    runs.reserve(4);
    for (const int threads : {1, 3, 8, 16}) {
        runs.emplace_back();
        Run& r = runs.back();
        r.params.init(Vector3d(-1, -1, -1), Vector3d(16, 16, 16));
        r.mesh = std::make_unique<TetWildMesh>(r.params, nullptr, threads);
        load(*r.mesh, grid);
        REQUIRE(r.mesh->use_colored_smoothing());
        r.mesh->smooth_all_vertices(2);
    }

    const TetWildMesh& ref = *runs.front().mesh;
    size_t moved = 0;
    for (size_t v = 0; v < grid.points.size(); ++v) {
        if (ref.m_vertex_attribute[v].m_posf != grid.points[v]) ++moved;
    }
    CHECK(moved > grid.points.size() / 4); // the passes really did something

    for (size_t r = 1; r < runs.size(); ++r) {
        const TetWildMesh& other = *runs[r].mesh;
        size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
        for (size_t v = 0; v < grid.points.size(); ++v) {
            const auto& a = ref.m_vertex_attribute[v];
            const auto& b = other.m_vertex_attribute[v];
            if (a.m_posf != b.m_posf) ++pos_diff;
            if (!(a.pos() == b.pos()) || a.m_is_rounded != b.m_is_rounded) ++exact_diff;
        }
        for (size_t t = 0; t < grid.tets.size(); ++t) {
            if (ref.m_tet_attribute[t].m_quality != other.m_tet_attribute[t].m_quality) {
                ++quality_diff;
            }
        }
        CHECK(pos_diff == 0);
        CHECK(exact_diff == 0);
        CHECK(quality_diff == 0);
    }
}

TEST_CASE(
    "colored smoothing with a surface does not depend on the number of threads",
    "[tetwild][smoothing]")
{
    // The same property where smoothing does the most: a slanted closed surface inserted through
    // the serial pipeline, so its vertices are smoothed against the envelope (projection and
    // containment check), and the arrangement leaves vertices whose exact position is not a
    // double, which smoothing has to round first.
    const std::vector<Vector3d> v = {
        {0.05, 0.02, 0.01},
        {1.13, 0.17, 0.29},
        {0.31, 1.07, 0.13},
        {0.23, 0.37, 1.11}};
    const std::vector<std::array<size_t, 3>> f = {
        {{0, 2, 1}},
        {{0, 1, 3}},
        {{0, 3, 2}},
        {{1, 2, 3}}};

    Parameters params;
    params.init(v, f);
    components::shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(v, 0);
    surf_mesh.create_mesh(v.size(), f, {}, params.eps);
    const std::shared_ptr<SampleEnvelope> env(&surf_mesh.m_envelope, [](SampleEnvelope*) {});

    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;
    std::vector<int8_t> tet_face_orientation;
    {
        TetWildMesh insertion(params, env, 0);
        insertion.insertion_by_volumeremesher(
            v,
            f,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation);
    }

    std::vector<std::unique_ptr<TetWildMesh>> meshes;
    for (const int threads : {1, 4, 16}) {
        meshes.push_back(std::make_unique<TetWildMesh>(params, env, threads));
        TetWildMesh& mesh = *meshes.back();
        mesh.init_from_Volumeremesher(
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation);
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

TEST_CASE("colored smoothing rejects a hook that writes a neighbour", "[tetwild][smoothing]")
{
    // Two vertices of a class can share a neighbour; a hook that reaches it through non-const
    // access has it written back when the smooth is rejected, by both threads at once. The
    // colored pass checks every smooth and throws on the first such access -- on any number of
    // threads, so the bug cannot hide behind a lucky schedule.
    const JitteredGrid grid = make_jittered_grid(6, 7);
    for (const int threads : {1, 4}) {
        CAPTURE(threads);
        Parameters params;
        params.init(Vector3d(-1, -1, -1), Vector3d(6, 6, 6));
        NeighbourReadingMesh mesh(params, nullptr, threads);
        load(mesh, grid);
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
    params.init(Vector3d(-1, -1, -1), Vector3d(6, 6, 6));
    params.colored_smoothing = false;
    NeighbourReadingMesh mesh(params, nullptr, 4);
    load(mesh, grid);
    REQUIRE_FALSE(mesh.use_colored_smoothing());
    REQUIRE_NOTHROW(mesh.smooth_all_vertices(1));
}
