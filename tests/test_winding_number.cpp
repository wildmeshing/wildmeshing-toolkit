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
