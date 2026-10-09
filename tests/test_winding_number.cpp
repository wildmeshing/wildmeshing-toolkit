#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/WindingNumber.hpp>

#include <igl/winding_number.h>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cmath>
#include <mutex>
#include <random>
#include <set>
#include <thread>

using namespace wmtk;

namespace {

// A closed polygon plus a second, disjoint one, so the soup has more than one loop and the
// winding number is not trivially constant.
void two_loops(Eigen::MatrixXd& V, Eigen::MatrixXi& E, int n_per_loop)
{
    V.resize(2 * n_per_loop, 2);
    E.resize(2 * n_per_loop, 2);
    for (int k = 0; k < 2; ++k) {
        const double cx = k == 0 ? 0.0 : 5.0;
        for (int i = 0; i < n_per_loop; ++i) {
            const double t = 2.0 * M_PI * i / n_per_loop;
            const int v = k * n_per_loop + i;
            V(v, 0) = cx + std::cos(t);
            V(v, 1) = std::sin(t);
            E(v, 0) = v;
            E(v, 1) = k * n_per_loop + (i + 1) % n_per_loop;
        }
    }
}

Eigen::MatrixXd query_grid(int n)
{
    Eigen::MatrixXd O(n * n, 2);
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> d(-2.0, 7.0);
    for (int i = 0; i < n * n; ++i) {
        O(i, 0) = d(gen);
        O(i, 1) = d(gen);
    }
    return O;
}

/// A UV sphere of radius 1 around the origin, 24 x 48 faces, closed unless `hemisphere`, in
/// which case only its upper half is kept: open, with the equator as boundary.
void uv_sphere(Eigen::MatrixXd& V, Eigen::MatrixXi& F, bool hemisphere = false)
{
    const int n_lat = 24, n_lon = 48;
    V.resize((n_lat - 1) * n_lon + 2, 3);
    V.row(0) << 0, 0, 1;
    V.row(1) << 0, 0, -1;
    for (int i = 1; i < n_lat; ++i) {
        for (int j = 0; j < n_lon; ++j) {
            const double th = M_PI * i / n_lat, ph = 2 * M_PI * j / n_lon;
            V.row(2 + (i - 1) * n_lon + j) << std::sin(th) * std::cos(ph),
                std::sin(th) * std::sin(ph), std::cos(th);
        }
    }
    const auto ring = [&](int i, int j) { return 2 + (i - 1) * n_lon + (j % n_lon); };
    std::vector<Eigen::RowVector3i> Fs;
    const int last = hemisphere ? n_lat / 2 : n_lat - 1;
    for (int j = 0; j < n_lon; ++j) {
        Fs.emplace_back(0, ring(1, j), ring(1, j + 1));
        if (!hemisphere) Fs.emplace_back(1, ring(n_lat - 1, j + 1), ring(n_lat - 1, j));
        for (int i = 1; i < last; ++i) {
            Fs.emplace_back(ring(i, j), ring(i + 1, j), ring(i + 1, j + 1));
            Fs.emplace_back(ring(i, j), ring(i + 1, j + 1), ring(i, j + 1));
        }
    }
    F.resize(Fs.size(), 3);
    for (size_t i = 0; i < Fs.size(); ++i) F.row(i) = Fs[i];
}

Eigen::MatrixXd random_points(int n, double extent, unsigned seed)
{
    Eigen::MatrixXd O(n, 3);
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> d(-extent, extent);
    for (Eigen::Index i = 0; i < O.rows(); ++i) O.row(i) << d(gen), d(gen), d(gen);
    return O;
}

} // namespace

TEST_CASE("WindingNumberHierarchy is igl's winding number", "[winding_number]")
{
    // igl's hierarchy, step for step, but for the apex of the fans closing its nodes, which igl
    // draws with rand(): the values agree to rounding, closed or open, and whatever duplicate
    // vertices the mesh carries (merged first, as igl does).
    for (const bool open : {false, true}) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        uv_sphere(V, F, open);
        // A duplicated vertex: the first face refers to a copy of vertex 0.
        V.conservativeResize(V.rows() + 1, 3);
        V.row(V.rows() - 1) = V.row(0);
        F(0, 0) = int(V.rows() - 1);
        const Eigen::MatrixXd O = random_points(5000, 1.5, open ? 5 : 4);
        Eigen::VectorXd W_igl;
        igl::winding_number(V, F, O, W_igl);
        const utils::WindingNumberHierarchy hier(V, F);
        double max_diff = 0;
        for (Eigen::Index i = 0; i < O.rows(); ++i) {
            max_diff = std::max(max_diff, std::abs(hier.winding_number(O.row(i)) - W_igl(i)));
        }
        INFO("open " << open);
        CHECK(max_diff < 1e-12);
    }
}

TEST_CASE("WindingNumberHierarchy is reproducible and owns its vertices", "[winding_number]")
{
    Eigen::MatrixXd V, V2;
    Eigen::MatrixXi F, F2;
    uv_sphere(V, F);
    uv_sphere(V2, F2);
    V2.col(0).array() += 0.5; // the same sphere, shifted
    const Eigen::MatrixXd O = random_points(5000, 1.5, 6);

    const utils::WindingNumberHierarchy a(V, F);
    Eigen::VectorXd W_a(O.rows());
    for (Eigen::Index i = 0; i < O.rows(); ++i) W_a(i) = a.winding_number(O.row(i));

    // Two builds of one mesh agree to the last bit: igl's drew each node's fan apex with rand().
    const utils::WindingNumberHierarchy a2(V, F);
    // Building another mesh's hierarchy leaves the first one alone: igl kept every tree's
    // vertices in one static, which the second build overwrote.
    const utils::WindingNumberHierarchy b(V2, F2);
    for (Eigen::Index i = 0; i < O.rows(); ++i) {
        REQUIRE(a2.winding_number(O.row(i)) == W_a(i));
        REQUIRE(a.winding_number(O.row(i)) == W_a(i));
    }
    // And b answers for its own sphere.
    CHECK(std::abs(b.winding_number(Eigen::RowVector3d(1.3, 0, 0)) - 1) < 1e-9);
    CHECK(std::abs(a.winding_number(Eigen::RowVector3d(1.3, 0, 0))) < 1e-9);
}


TEST_CASE("winding_number_2d matches igl bit for bit", "[winding_number]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    two_loops(V, E, 64);

    // Deliberately more than igl's min_parallel of 10000, so igl's own reference call takes
    // its parallel path -- the branch the wmtk version replaces.
    const Eigen::MatrixXd O = query_grid(120);
    REQUIRE(O.rows() > 10000);

    Eigen::VectorXd W_igl;
    igl::winding_number(V, E, O, W_igl);

    for (int nt : {0, 1, 4, 8}) {
        Eigen::VectorXd W;
        utils::winding_number_2d(V, E, O, W, nt);
        REQUIRE(W.rows() == W_igl.rows());
        for (Eigen::Index i = 0; i < W.rows(); ++i) {
            // Bit-identical, not approximate: same formula, same accumulation order.
            REQUIRE(W(i) == W_igl(i));
        }
    }
}

TEST_CASE("winding_number_2d is inside/outside correct", "[winding_number]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    two_loops(V, E, 128);

    Eigen::MatrixXd O(4, 2);
    O << 0.0, 0.0, // inside loop 0
        5.0, 0.0, // inside loop 1
        2.5, 0.0, // between them
        100.0, 100.0; // far away

    Eigen::VectorXd W;
    utils::winding_number_2d(V, E, O, W, 4);

    REQUIRE(std::abs(W(0)) > 0.5);
    REQUIRE(std::abs(W(1)) > 0.5);
    REQUIRE(std::abs(W(2)) < 0.5);
    REQUIRE(std::abs(W(3)) < 0.5);
}

TEST_CASE("winding_number_2d honours num_threads", "[winding_number]")
{
    // The regression this file exists for. winding_number_2d used to hand each of its chunks
    // to igl::winding_number(V, E, O, W), which for a segment soup runs its own
    // igl::parallel_for over igl::default_num_threads() -- hardware_concurrency() -- once the
    // chunk exceeds 10000 rows. The two counts multiplied: 8 requested threads became 8 x 128
    // live threads on a 128-core machine.
    //
    // Counting threads directly is unreliable (the OS reuses them, and other libraries have
    // pools), so count how many DISTINCT threads run the user-visible work instead. Only
    // wmtk's parallel_for may create them, so the distinct count is bounded by num_threads.
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    two_loops(V, E, 32);
    const Eigen::MatrixXd O = query_grid(150); // 22500 rows: over igl's 10000 threshold

    for (int nt : {1, 2, 4}) {
        std::mutex m;
        std::set<std::thread::id> ids;
        Eigen::VectorXd W;
        W.setZero(O.rows());
        threading::parallel_for(
            threading::range(0, static_cast<size_t>(O.rows())),
            [&](const threading::range& r) {
                {
                    std::lock_guard<std::mutex> lock(m);
                    ids.insert(std::this_thread::get_id());
                }
                for (size_t o = r.begin(); o < r.end(); ++o) {
                    const Eigen::Index i = static_cast<Eigen::Index>(o);
                    W(i) = utils::winding_number_2d_point(V, E, O(i, 0), O(i, 1));
                }
            },
            nt);
        REQUIRE(ids.size() <= static_cast<size_t>(nt));
    }
}

TEST_CASE("winding numbers do not depend on the number of threads", "[winding_number]")
{
    // The queries are handed out in chunks on demand, so which thread evaluates which query
    // depends on the schedule. Each query is evaluated on its own, so the values must not:
    // bit for bit the same on any number of threads.
    SECTION("3D")
    {
        // A closed UV sphere with enough faces for the hierarchy to grow below its root.
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        uv_sphere(V, F);

        const Eigen::MatrixXd O = random_points(20000, 1.5, 3);

        Eigen::VectorXd W1, W3, W8;
        utils::winding_number(V, F, O, W1, 1);
        utils::winding_number(V, F, O, W3, 3);
        utils::winding_number(V, F, O, W8, 8);
        CHECK(W1 == W3);
        CHECK(W1 == W8);
        // And it is a winding number: 1 inside the sphere, 0 outside, away from it.
        for (Eigen::Index i = 0; i < O.rows(); ++i) {
            const double r = O.row(i).norm();
            if (r < 0.9) CHECK(std::abs(W1(i) - 1) < 1e-9);
            if (r > 1.1) CHECK(std::abs(W1(i)) < 1e-9);
        }
    }
    SECTION("2D")
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi E;
        two_loops(V, E, 64);
        const Eigen::MatrixXd O = query_grid(120);
        Eigen::VectorXd W1, W3, W8;
        utils::winding_number_2d(V, E, O, W1, 1);
        utils::winding_number_2d(V, E, O, W3, 3);
        utils::winding_number_2d(V, E, O, W8, 8);
        CHECK(W1 == W3);
        CHECK(W1 == W8);
    }
}
