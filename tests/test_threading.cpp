#include <catch2/catch_test_macros.hpp>

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <atomic>
#include <cfenv>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <wmtk/Types.hpp>
#include <wmtk/threading/collector.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/indexed_collector.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/threading/spin_mutex.hpp>
#include <wmtk/threading/task_group.hpp>
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

TEST_CASE("threading_collector", "[threading]")
{
    threading::collector<size_t> c;
    threading::indexed_collector<size_t> ic(100);

    threading::parallel_for(
        threading::range(0, 100),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); ++i) {
                c.push_back(i);
                ic.set(i, i);
            }
        },
        10);

    REQUIRE(c.size() == 100);
    std::vector<bool> seen(100, false);
    for (size_t i = 0; i < c.size(); ++i) {
        REQUIRE((c[i] >= 0 && c[i] < 100));
        seen[c[i]] = true;
    }
    for (size_t i = 0; i < seen.size(); ++i) {
        CHECK(seen[i]);
    }

    const auto compact_ic = ic.compact();
    REQUIRE(compact_ic.size() == 100);
    for (size_t i = 0; i < compact_ic.size(); ++i) {
        REQUIRE(compact_ic[i] == i);
    }
}

TEST_CASE("enumerable_thread_specific", "[threading]")
{
    threading::enumerable_thread_specific<size_t> ets;
    threading::collector<size_t> c;

    constexpr size_t N = 100;
    const size_t num_threads = 4;

    threading::parallel_for(
        threading::range(0, N),
        [&](const threading::range& r) {
            ets.local() = 0;
            for (size_t i = r.begin(); i < r.end(); ++i) {
                ets.local() += i;
            }
            c.push_back(ets.local());
        },
        num_threads);

    REQUIRE(c.size() == num_threads);

    size_t total_sum = 0;
    for (const size_t v : c) {
        total_sum += v;
    }

    CHECK(total_sum == (N * (N - 1)) / 2);
}

namespace {
// Run `n` tasks that each call `fn` and then wait until all `n` have started, so they are
// guaranteed to occupy `n` distinct workers. With `n` larger than any other test asks for, that
// is every worker in the pool: which idle worker picks up a task is otherwise arbitrary.
// Returns false if the tasks did not all start within the deadline.
template <typename F>
bool run_on_distinct_workers(int n, F fn)
{
    std::atomic<int> started{0};
    std::atomic<bool> all_met{true};
    threading::task_group tg;
    for (int i = 0; i < n; ++i) {
        tg.run([&]() {
            fn();
            started.fetch_add(1);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
            while (started.load() < n) {
                if (std::chrono::steady_clock::now() > deadline) {
                    all_met = false;
                    return;
                }
                std::this_thread::yield();
            }
        });
    }
    tg.wait();
    return all_met;
}
// Larger than the concurrency any other test in this file uses, so a wave this wide covers the
// whole pool (the pool only grows to the largest number of tasks ever outstanding at once).
constexpr int kWholePool = 128;
} // namespace

TEST_CASE("task_group", "[threading]")
{
    SECTION("runs every task, and wait() can be called again")
    {
        std::atomic<int> sum{0};
        threading::task_group tg;
        for (int round = 0; round < 3; ++round) {
            for (int i = 1; i <= 10; ++i) {
                tg.run([&sum, i]() { sum.fetch_add(i); });
            }
            tg.wait();
            CHECK(sum.load() == 55 * (round + 1));
        }
    }

    SECTION("rethrows the first exception, and the group stays usable")
    {
        threading::task_group tg;
        tg.run([]() { throw std::runtime_error("boom"); });
        tg.run([]() {});
        REQUIRE_THROWS_AS(tg.wait(), std::runtime_error);
        std::atomic<bool> ran{false};
        tg.run([&ran]() { ran = true; });
        tg.wait();
        CHECK(ran);
    }

    SECTION("every task runs concurrently with the others")
    {
        // A pool that queued tasks behind running ones would never let this finish: each task
        // waits until all of them have started. The deadline turns a hang into a failure.
        constexpr int kTasks = 24;
        std::atomic<int> started{0};
        std::atomic<bool> all_met{true};
        threading::task_group tg;
        for (int i = 0; i < kTasks; ++i) {
            tg.run([&]() {
                started.fetch_add(1);
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
                while (started.load() < kTasks) {
                    if (std::chrono::steady_clock::now() > deadline) {
                        all_met = false;
                        return;
                    }
                    std::this_thread::yield();
                }
            });
        }
        tg.wait();
        CHECK(all_met);
    }

    SECTION("a task can run a task_group of its own")
    {
        std::atomic<int> inner{0};
        threading::task_group outer;
        for (int i = 0; i < 4; ++i) {
            outer.run([&inner]() {
                threading::task_group tg;
                for (int j = 0; j < 4; ++j) {
                    tg.run([&inner]() { inner.fetch_add(1); });
                }
                tg.wait();
            });
        }
        outer.wait();
        CHECK(inner.load() == 16);
    }

    SECTION("workers are reused across groups")
    {
        // The point of the pool: a later pass runs on threads that already ran tasks, and so
        // with their thread_local caches.
        static thread_local int tasks_run_here = 0;
        REQUIRE(run_on_distinct_workers(kWholePool, []() { ++tasks_run_here; }));
        // Give the workers time to get back to waiting; otherwise the pool may (correctly)
        // start new ones rather than wait for them.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::atomic<int> on_used_thread{0};
        {
            threading::task_group tg;
            for (int i = 0; i < 4; ++i) {
                tg.run([&on_used_thread]() {
                    if (tasks_run_here++ > 0) on_used_thread.fetch_add(1);
                });
            }
            tg.wait();
        }
        CHECK(on_used_thread.load() > 0);
    }

    SECTION("a task starts in round-to-nearest even if an earlier one did not restore it")
    {
        threading::task_group tg;
        for (int round = 0; round < 8; ++round) {
            tg.run([]() { std::fesetround(FE_UPWARD); });
            tg.wait();
            int mode = -1;
            tg.run([&mode]() { mode = std::fegetround(); });
            tg.wait();
            CHECK(mode == FE_TONEAREST);
        }
    }
}

namespace {
// Counts destructions of live values only: local() builds its value through a by-value
// factory, so moved-from temporaries are destroyed along the way and must not count.
struct CountsDestructions
{
    static std::atomic<int>& destroyed()
    {
        static std::atomic<int> n{0};
        return n;
    }
    static std::atomic<int>& wrong_thread()
    {
        static std::atomic<int> n{0};
        return n;
    }
    std::thread::id created_on = std::this_thread::get_id();
    bool live = true;

    CountsDestructions() = default;
    CountsDestructions(const CountsDestructions&) = delete;
    CountsDestructions(CountsDestructions&& o) noexcept
        : created_on(o.created_on)
        , live(o.live)
    {
        o.live = false;
    }
    ~CountsDestructions()
    {
        if (!live) return;
        destroyed().fetch_add(1);
        if (std::this_thread::get_id() != created_on) wrong_thread().fetch_add(1);
    }
};
} // namespace

TEST_CASE("enumerable_thread_specific_slots_of_dead_instances", "[threading]")
{
    // task_group workers persist, so a slot outlives its instance on every worker that used
    // it. Each worker must drop such slots itself, on its own thread, the next time it creates
    // a slot -- otherwise every mesh ever built leaves its per-thread caches behind for good.
    CountsDestructions::destroyed() = 0;
    CountsDestructions::wrong_thread() = 0;
    // Let earlier tests' workers go idle, so the wave below is spread over exactly
    // kWholePool workers and the second wave meets every one of them again.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
        threading::enumerable_thread_specific<CountsDestructions> ets;
        REQUIRE(run_on_distinct_workers(kWholePool, [&ets]() { ets.local(); }));
    }
    // The instance is gone, its values are not: they live on the workers until those meet a
    // new instance.
    CHECK(CountsDestructions::destroyed().load() == 0);

    // Every worker meets a new instance, and so drops its dead slot.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    threading::enumerable_thread_specific<CountsDestructions> fresh;
    REQUIRE(run_on_distinct_workers(kWholePool, [&fresh]() { fresh.local(); }));

    // Every dead value, each destroyed on the thread that created it. (`fresh`'s own values are
    // still alive on the workers, so they do not count.)
    CHECK(CountsDestructions::destroyed().load() == kWholePool);
    CHECK(CountsDestructions::wrong_thread().load() == 0);
}

TEST_CASE("spin_mutex", "[threading]")
{
    SECTION("mutual exclusion")
    {
        threading::spin_mutex m;
        // Deliberately not atomic: the mutex is the thing under test, so the counter has to
        // be unprotected by anything else for a lost update to be observable.
        size_t counter = 0;
        constexpr int kThreads = 8;
        constexpr int kIters = 10000;

        threading::task_group tg;
        for (int t = 0; t < kThreads; ++t) {
            tg.run([&m, &counter]() {
                for (int i = 0; i < kIters; ++i) {
                    m.lock();
                    ++counter;
                    m.unlock();
                }
            });
        }
        tg.wait();

        CHECK(counter == size_t(kThreads) * kIters);
    }

    SECTION("try_lock fails while held")
    {
        threading::spin_mutex m;
        REQUIRE(m.try_lock());
        // Not recursive: a second attempt fails even from the owning thread. The two-ring
        // walks depend on this -- it is why they track an owner id at all.
        CHECK_FALSE(m.try_lock());
        m.unlock();
        CHECK(m.try_lock());
        m.unlock();
    }
}

TEST_CASE("vertex_mutex_owner_integrity", "[threading]")
{
    // The invariant the two-ring walks rely on: while a thread holds a vertex, that vertex's
    // owner field reads back as that thread's id. The walks use `get_owner() == threadid` to
    // skip vertices they claimed earlier in the same acquisition; if the field can go stale
    // under them they re-attempt a lock they already hold, fail (spin_mutex is not recursive)
    // and abort an operation that should have succeeded.
    //
    // The unlocked `get_owner()` read below is not incidental -- it is exactly what the walks
    // do, and it is what made a non-atomic owner field a data race.
    //
    // Note on what this test can and cannot catch. The bug it guards against (releasing the
    // mutex before clearing the owner) had a window about one instruction wide, so a plain
    // Release build does not reproduce it by timing: measured 0 violations in 32k acquisitions
    // against the broken code. Under ThreadSanitizer, which perturbs scheduling, the same loop
    // gave 24 violations in 28k acquisitions and flagged five data races on the field. So this
    // is a regression guard and a TSan vehicle, not a standalone reproducer -- build the tests
    // with -DSANITIZE_THREAD=ON for it to have real detection power.
    using VertexMutex = wmtk::TetMesh::VertexMutex;

    constexpr int kThreads = 8;
    constexpr int kIters = 4000;
    constexpr size_t kSlots = 4; // few slots, heavy contention

    std::vector<VertexMutex> mutexes(kSlots);
    std::atomic<int> violations{0};
    std::atomic<size_t> acquisitions{0};

    threading::task_group tg;
    for (int id = 0; id < kThreads; ++id) {
        tg.run([&mutexes, &violations, &acquisitions, id]() {
            for (int it = 0; it < kIters; ++it) {
                VertexMutex& m = mutexes[size_t(it) % kSlots];

                // The walk's fast path: an unlocked read of a vertex another thread may own.
                if (m.get_owner() == id) {
                    continue;
                }
                if (!m.trylock()) {
                    continue;
                }
                m.set_owner(id);
                acquisitions.fetch_add(1, std::memory_order_relaxed);

                // Hold briefly. yield() is opaque to the optimizer, so the re-read below is a
                // real reload rather than a hoisted copy of the store above.
                std::this_thread::yield();

                if (m.get_owner() != id) {
                    violations.fetch_add(1, std::memory_order_relaxed);
                }
                m.unlock();
            }
        });
    }
    tg.wait();

    // Guard against a vacuous pass: the test is only meaningful if locks were actually taken.
    REQUIRE(acquisitions.load() > 0);
    CHECK(violations.load() == 0);
}

TEST_CASE("vertex_mutex_two_ring_no_leak", "[threading]")
{
    // A fan of five tets around the edge (0,1): every edge's two-ring covers the whole mesh,
    // so concurrent acquisitions are guaranteed to collide and exercise the abort path.
    TetMesh mesh;
    mesh.init(7, {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 6}}, {{0, 1, 6, 2}}});

    const auto edges = mesh.get_edges();
    REQUIRE(!edges.empty());

    constexpr int kThreads = 8;
    constexpr int kIters = 2000;
    std::atomic<size_t> acquired{0};
    std::atomic<size_t> aborted{0};

    threading::task_group tg;
    for (int id = 0; id < kThreads; ++id) {
        tg.run([&mesh, &edges, &acquired, &aborted, id]() {
            for (int it = 0; it < kIters; ++it) {
                const auto& e = edges[size_t(it) % edges.size()];
                if (mesh.try_set_edge_mutex_two_ring(e, id)) {
                    acquired.fetch_add(1, std::memory_order_relaxed);
                } else {
                    aborted.fetch_add(1, std::memory_order_relaxed);
                }
                // Both paths must release: a failed acquisition still leaves partial locks
                // on the stack, and dropping them is what the scheduler's cleanup does.
                mesh.release_vertex_mutex_in_stack();
            }
        });
    }
    tg.wait();

    REQUIRE(acquired.load() > 0);
    INFO("acquired " << acquired.load() << ", aborted " << aborted.load());

    // Nothing may stay locked once every thread has released. If any vertex leaked, at least
    // one of these single-threaded acquisitions cannot complete.
    for (const auto& e : edges) {
        CHECK(mesh.try_set_edge_mutex_two_ring(e, 0));
        mesh.release_vertex_mutex_in_stack();
    }
}