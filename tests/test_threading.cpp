#include <catch2/catch_test_macros.hpp>

#include <igl/Timer.h>
#include <stdexcept>
#include <wmtk/Types.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>

using namespace wmtk;

TEST_CASE("parallel_for", "[threading]")
{
    SECTION("ID check")
    {
        std::vector<int> v(1000, 0);
        threading::parallel_for(
            threading::range(0, v.size()),
            [&](const threading::range& r) {
                for (size_t i = r.begin(); i < r.end(); ++i) {
                    v[i] = i;
                }
            },
            10);
        for (size_t i = 0; i < v.size(); ++i) {
            REQUIRE(v[i] == i);
        }
    }

    SECTION("rethrows the first exception")
    {
        REQUIRE_THROWS_AS(
            threading::parallel_for(
                threading::range(0, 8),
                [&](const threading::range& r) {
                    if (r.begin() == 0) {
                        throw std::runtime_error("parallel_for failure");
                    }
                },
                10),
            std::runtime_error);
    }

    SECTION("negative range")
    {
        threading::parallel_for(
            threading::range(0, 0),
            [](const threading::range& r) {
                REQUIRE(false); // should not be called
            },
            10);

        threading::parallel_for(
            threading::range(2, 0),
            [](const threading::range& r) {
                REQUIRE(false); // should not be called
            },
            10);
    }
}

TEST_CASE("parallel_for_performance", "[threading][.]")
{
    /**
     * For testing the performance of parallel_for.
     */

    igl::Timer timer;

    SECTION("vector sum")
    {
        logger().info("=== vector sum ===");

        constexpr size_t N = 1000000;
        VectorXd a = VectorXd::Random(N);
        VectorXd b = VectorXd::Random(N);
        VectorXd c = VectorXd::Zero(N);

        auto sum_func = [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                c[i] = a[i] + b[i];
            }
        };

        // serial
        timer.start();
        for (size_t i = 0; i < c.size(); ++i) {
            c[i] = a[i] + b[i];
        }
        timer.stop();
        double duration_serial = timer.getElapsedTimeInMilliSec();
        logger().info("serial duration: {} ms", duration_serial);

        auto parallel_execute = [&](int num_threads) {
            timer.start();
            threading::parallel_for(threading::range(0, c.size()), sum_func, num_threads);
            timer.stop();

            double duration = timer.getElapsedTimeInMilliSec();
            logger().info(
                "parallel duration ({} threads): {} ms; speedup: {}",
                num_threads,
                duration,
                duration_serial / duration);
            return duration;
        };

        parallel_execute(1);
        parallel_execute(2);
        parallel_execute(4);
        parallel_execute(8);
        parallel_execute(16);
    }
    SECTION("matrix-vector multiplication")
    {
        logger().info("=== matrix-vector multiplication ===");

        constexpr size_t N = 5000;
        VectorXd a = VectorXd::Random(N);
        VectorXd c = VectorXd::Zero(N);
        MatrixXd A = MatrixXd::Random(N, N);

        auto matmul_func = [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                c[i] = A.row(i).dot(a);
            }
        };

        // serial
        timer.start();
        for (size_t i = 0; i < c.size(); ++i) {
            c[i] = A.row(i).dot(a);
        }
        timer.stop();
        double duration_serial = timer.getElapsedTimeInMilliSec();
        logger().info("serial duration: {} ms", duration_serial);

        auto parallel_execute = [&](int num_threads) {
            timer.start();
            threading::parallel_for(threading::range(0, c.size()), matmul_func, num_threads);
            timer.stop();

            double duration = timer.getElapsedTimeInMilliSec();
            logger().info(
                "parallel duration ({} threads): {} ms; speedup: {}",
                num_threads,
                duration,
                duration_serial / duration);
            return duration;
        };

        parallel_execute(1);
        parallel_execute(2);
        parallel_execute(4);
        parallel_execute(8);
        parallel_execute(16);
    }
}