#include <wmtk/utils/WindingNumber.hpp>

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

} // namespace

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

namespace {

/// Axis-aligned box as 12 triangles appended to (V, F), outward-facing unless `inward`.
void add_box(
    std::vector<Eigen::RowVector3d>& V,
    std::vector<Eigen::RowVector3i>& F,
    const Eigen::RowVector3d& lo,
    const Eigen::RowVector3d& hi,
    bool inward)
{
    const int base = int(V.size());
    for (int i = 0; i < 8; ++i) {
        V.emplace_back(i & 1 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 4 ? hi[2] : lo[2]);
    }
    const int tris[12][3] = {
        {0, 2, 3},
        {0, 3, 1},
        {4, 5, 7},
        {4, 7, 6},
        {0, 1, 5},
        {0, 5, 4},
        {2, 6, 7},
        {2, 7, 3},
        {0, 4, 6},
        {0, 6, 2},
        {1, 3, 7},
        {1, 7, 5}};
    for (const auto& t : tris) {
        if (inward) {
            F.emplace_back(base + t[0], base + t[2], base + t[1]);
        } else {
            F.emplace_back(base + t[0], base + t[1], base + t[2]);
        }
    }
}

} // namespace

TEST_CASE("winding_number_by_group adds up to the whole mesh", "[winding_number]")
{
    // Three closed groups: a box, a box nested in it and oriented inward (a cavity), and a box
    // beside them. The winding number is additive over faces, so the groups' sum must be the
    // whole mesh's, which is what tetwild's finalization uses instead of evaluating the whole.
    std::vector<Eigen::RowVector3d> Vs;
    std::vector<Eigen::RowVector3i> Fs;
    std::vector<int> group;
    add_box(Vs, Fs, {0, 0, 0}, {4, 4, 4}, false);
    group.resize(Fs.size(), 0);
    add_box(Vs, Fs, {1, 1, 1}, {3, 3, 3}, true);
    group.resize(Fs.size(), 1);
    add_box(Vs, Fs, {6, 0, 0}, {8, 2, 2}, false);
    group.resize(Fs.size(), 2);
    Eigen::MatrixXd V(Vs.size(), 3);
    for (size_t i = 0; i < Vs.size(); ++i) V.row(i) = Vs[i];
    Eigen::MatrixXi F(Fs.size(), 3);
    for (size_t i = 0; i < Fs.size(); ++i) F.row(i) = Fs[i];

    Eigen::MatrixXd O(2000, 3);
    std::mt19937 gen(7);
    std::uniform_real_distribution<double> d(-1.0, 9.0);
    for (Eigen::Index i = 0; i < O.rows(); ++i) O.row(i) << d(gen), d(gen) / 2, d(gen) / 2;

    Eigen::VectorXd W_whole;
    utils::winding_number(V, F, O, W_whole, 4);
    Eigen::MatrixXd W_groups;
    utils::winding_number_by_group(V, F, group, 3, O, W_groups, 4);
    REQUIRE(W_groups.rows() == O.rows());
    REQUIRE(W_groups.cols() == 3);

    CHECK((W_groups.rowwise().sum() - W_whole).cwiseAbs().maxCoeff() < 1e-9);

    for (int g = 0; g < 3; ++g) {
        std::vector<int> rows;
        for (size_t f = 0; f < group.size(); ++f) {
            if (group[f] == g) rows.push_back(int(f));
        }
        Eigen::MatrixXi Fg(rows.size(), 3);
        for (size_t i = 0; i < rows.size(); ++i) Fg.row(i) = F.row(rows[i]);
        Eigen::VectorXd W_g;
        utils::winding_number(V, Fg, O, W_g, 4);
        CHECK((W_groups.col(g) - W_g).cwiseAbs().maxCoeff() < 1e-9);
    }

    // The cavity: inside the outer box and the inner one, the whole mesh's winding number is 0.
    Eigen::MatrixXd center(1, 3);
    center << 2, 2, 2;
    Eigen::MatrixXd W_center;
    utils::winding_number_by_group(V, F, group, 3, center, W_center, 1);
    CHECK(std::abs(W_center.sum()) < 1e-9);
    CHECK(std::abs(W_center(0, 1) + 1) < 1e-9);
}

TEST_CASE("reversing every face negates the winding number", "[winding_number]")
{
    // tetwild orients an inside-out input by negating its winding number rather than
    // evaluating the reversed surface again.
    std::vector<Eigen::RowVector3d> Vs;
    std::vector<Eigen::RowVector3i> Fs;
    add_box(Vs, Fs, {0, 0, 0}, {4, 4, 4}, false);
    add_box(Vs, Fs, {6, 0, 0}, {8, 2, 2}, false);
    Eigen::MatrixXd V(Vs.size(), 3);
    for (size_t i = 0; i < Vs.size(); ++i) V.row(i) = Vs[i];
    Eigen::MatrixXi F(Fs.size(), 3);
    for (size_t i = 0; i < Fs.size(); ++i) F.row(i) = Fs[i];
    Eigen::MatrixXi F_reversed = F;
    F_reversed.col(1).swap(F_reversed.col(2));

    Eigen::MatrixXd O(1000, 3);
    std::mt19937 gen(11);
    std::uniform_real_distribution<double> d(-1.0, 9.0);
    for (Eigen::Index i = 0; i < O.rows(); ++i) O.row(i) << d(gen), d(gen) / 2, d(gen) / 2;

    Eigen::VectorXd W, W_reversed;
    utils::winding_number(V, F, O, W, 2);
    utils::winding_number(V, F_reversed, O, W_reversed, 2);
    CHECK((W + W_reversed).cwiseAbs().maxCoeff() < 1e-12);
}

TEST_CASE("winding-number inside test and orientation", "[winding_number]")
{
    // The one inside test of tetwild, triwild and simwild: strictly above 1/2, so a point on
    // the surface is not inside.
    CHECK_FALSE(utils::winding_number_inside(0.5));
    CHECK(utils::winding_number_inside(0.5 + 1e-12));
    CHECK(utils::winding_number_inside(2.0));
    CHECK_FALSE(utils::winding_number_inside(-1.0));

    SECTION("nothing inside: taken as inside out and negated")
    {
        Eigen::VectorXd W(3);
        W << -1.0, -0.2, 0.5;
        CHECK(utils::orient_winding_number(W));
        CHECK(W(0) == 1.0);
        CHECK(W(2) == -0.5);
        CHECK(utils::any_winding_number_inside(W));
    }
    SECTION("something inside: left as it is")
    {
        Eigen::VectorXd W(2);
        W << 0.0, 1.0;
        CHECK_FALSE(utils::orient_winding_number(W));
        CHECK(W(1) == 1.0);
    }
    SECTION("no query point: left as it is")
    {
        Eigen::VectorXd W;
        CHECK_FALSE(utils::orient_winding_number(W));
        CHECK_FALSE(utils::any_winding_number_inside(W));
    }
}
