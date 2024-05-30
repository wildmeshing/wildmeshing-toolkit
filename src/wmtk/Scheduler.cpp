#include "Scheduler.hpp"

#include "Mesh.hpp"
#include <cassert>

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/simplex/k_ring.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/random_seed.hpp>
#include <wmtk/multimesh/utils/check_map_valid.hpp>

#include <polysolve/Utils.hpp>

#include <tbb/parallel_for.h>
// #include <tbb/task_arena.h>
#include <atomic>


#include <algorithm>
#include <random>

namespace wmtk {

Scheduler::Scheduler() = default;
Scheduler::~Scheduler() = default;

SchedulerStats Scheduler::run_operation_on_all(operations::Operation& op)
{
    SchedulerStats res;
    std::vector<simplex::Simplex> simplices;
    // op.reserve_enough_simplices();

    const auto type = op.primitive_type();
    {
        POLYSOLVE_SCOPED_STOPWATCH("Collecting primitives", res.collecting_time, logger());

        const auto tups = op.mesh().get_all(type);
        simplices =
            wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(op.mesh(), tups, type);
    }

    logger().debug("Executing on {} simplices", simplices.size());
    std::vector<std::pair<int64_t, double>> order;

    {
        POLYSOLVE_SCOPED_STOPWATCH("Sorting", res.sorting_time, logger());
        if (op.use_random_priority()) {
            std::mt19937 gen(utils::get_random_seed());

            std::shuffle(simplices.begin(), simplices.end(), gen);
        } else {
            order.reserve(simplices.size());
            for (int64_t i = 0; i < simplices.size(); ++i) {
                order.emplace_back(i, op.priority(simplices[i]));
            }

            std::stable_sort(order.begin(), order.end(), [](const auto& s_a, const auto& s_b) {
                return s_a.second < s_b.second;
            });
        }
    }


    {
        POLYSOLVE_SCOPED_STOPWATCH("Executing operation", res.executing_time, logger());


        if (op.use_random_priority()) {
            for (const auto& s : simplices) {
                auto mods = op(s);
                if (mods.empty())
                    res.fail();
                else
                    res.succeed();
            }
        } else {
            for (const auto& o : order) {
                auto mods = op(simplices[o.first]);
                if (mods.empty())
                    res.fail();
                else
                    res.succeed();
            }
        }
    }

    // logger().debug(
    //     "Ran {} ops, {} succeeded, {} failed",
    //     res.number_of_performed_operations(),
    //     res.number_of_successful_operations(),
    //     res.number_of_failed_operations());

    m_stats += res;

    return res;
}

SchedulerStats Scheduler::run_operation_on_all(
    operations::Operation& op,
    const TypedAttributeHandle<char>& flag_handle)
{
    std::vector<simplex::Simplex> simplices;
    const auto type = op.primitive_type();

    auto flag_accessor = op.mesh().create_accessor(flag_handle);
    auto tups = op.mesh().get_all(type);
    for (const auto& t : tups) {
        flag_accessor.scalar_attribute(t) = char(0);
    }

    SchedulerStats res = run_operation_on_all(op);
    int64_t success = res.number_of_successful_operations();
    std::vector<std::pair<int64_t, double>> order;

    while (success > 0) {
        SchedulerStats internal_stats;
        // op.reserve_enough_simplices();

        {
            POLYSOLVE_SCOPED_STOPWATCH(
                "Collecting primitives",
                internal_stats.collecting_time,
                logger());

            tups = op.mesh().get_all(type);
            const auto n_primitives = tups.size();
            tups.erase(
                std::remove_if(
                    tups.begin(),
                    tups.end(),
                    [&](const Tuple& t) { return flag_accessor.scalar_attribute(t) == char(0); }),
                tups.end());
            for (const auto& t : tups) {
                flag_accessor.scalar_attribute(t) = char(0);
            }

            logger().debug("Processing {}/{}", tups.size(), n_primitives);

            simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
                op.mesh(),
                tups,
                type);
        }

        {
            POLYSOLVE_SCOPED_STOPWATCH("Sorting", internal_stats.sorting_time, logger());
            if (op.use_random_priority()) {
                std::mt19937 gen(utils::get_random_seed());

                std::shuffle(simplices.begin(), simplices.end(), gen);
            } else {
                order.clear();
                order.reserve(simplices.size());
                for (int64_t i = 0; i < simplices.size(); ++i) {
                    order.emplace_back(i, op.priority(simplices[i]));
                }

                std::stable_sort(order.begin(), order.end(), [](const auto& s_a, const auto& s_b) {
                    return s_a.second < s_b.second;
                });
            }
        }

        {
            POLYSOLVE_SCOPED_STOPWATCH(
                "Executing operation",
                internal_stats.executing_time,
                logger());

            if (op.use_random_priority()) {
                for (const auto& s : simplices) {
                    auto mods = op(s);
		    //assert(wmtk::multimesh::utils::check_child_maps_valid(op.mesh()));
                    if (mods.empty()) {
                        res.fail();
		    } else {
                        res.succeed(); 
		    }
                }
            } else {
                for (const auto& o : order) {
                    auto mods = op(simplices[o.first]);
		    //assert(wmtk::multimesh::utils::check_child_maps_valid(op.mesh()));
                    if (mods.empty()) {
                        internal_stats.fail();
		    } else {
                        internal_stats.succeed();
		    }
                }
            }
        }

        success = internal_stats.number_of_successful_operations();
        res += internal_stats;
        res.sub_stats.push_back(internal_stats);
        m_stats += internal_stats;
        m_stats.sub_stats.push_back(internal_stats);
    }

    // // reset flag to 1, not necessaty
    // auto tups = op.mesh().get_all(type);
    // for (const auto& t : tups) {
    //     flag_accessor.scalar_attribute(t) = char(1);
    // }

    return res;
}

int64_t first_available_color(std::vector<int64_t>& used_neighbor_coloring)
{
    if (used_neighbor_coloring.size() == 0) return 0;

    std::sort(used_neighbor_coloring.begin(), used_neighbor_coloring.end());
    int64_t color = 0;

    for (const auto c : used_neighbor_coloring) {
        if (color < c) {
            return color;
        } else {
            color = c + 1;
        }
    }
    return color;
}

SchedulerStats Scheduler::run_operation_on_all_coloring(
    operations::Operation& op,
    const TypedAttributeHandle<int64_t>& color_handle)
{
    // this only works on vertex operations
    SchedulerStats res;
    std::vector<std::vector<simplex::Simplex>> colored_simplices;
    // op.reserve_enough_simplices();
    auto color_accessor = op.mesh().create_accessor<int64_t>(color_handle);

    const auto type = op.primitive_type();
    assert(type == PrimitiveType::Vertex);

    const auto tups = op.mesh().get_all(type);
    int64_t color_max = -1;
    {
        POLYSOLVE_SCOPED_STOPWATCH("Collecting primitives", res.collecting_time, logger());


        // reset all coloring as -1
        // TODO: parallelfor
        // for (const auto& v : tups) {
        //     color_accessor.scalar_attribute(v) = -1;
        // }

        tbb::parallel_for(
            tbb::blocked_range<int64_t>(0, tups.size()),
            [&](tbb::blocked_range<int64_t> r) {
                for (int64_t i = r.begin(); i < r.end(); ++i) {
                    color_accessor.scalar_attribute(tups[i]) = -1;
                }
            });

        // greedy coloring
        std::vector<int64_t> used_colors;
        for (const auto& v : tups) {
            // get used colors in neighbors
            used_colors.clear();
            for (const auto& v_one_ring :
                 simplex::link(op.mesh(), simplex::Simplex::vertex(op.mesh(), v), false)
                     .simplex_vector(PrimitiveType::Vertex)) {
                int64_t color = color_accessor.const_scalar_attribute(v_one_ring.tuple());
                if (color > -1) {
                    used_colors.push_back(color);
                }
            }
            int64_t c = first_available_color(used_colors);
            color_accessor.scalar_attribute(v) = c;
            color_max = std::max(color_max, c);

            // push into vectors
            if (c + 1 > colored_simplices.size()) {
                colored_simplices.push_back({simplex::Simplex::vertex(op.mesh(), v)});
            } else {
                colored_simplices[c].push_back(simplex::Simplex::vertex(op.mesh(), v));
            }
        }

        logger().info("Have {} colors among {} vertices", colored_simplices.size(), tups.size());

        // debug code

        {
            for (const auto& v : tups) {
                auto current_color = color_accessor.const_scalar_attribute(v);
                if (current_color == -1) {
                    std::cout << "vertex not assigned color!!!" << std::endl;
                }

                for (const auto& v_one_ring :
                     simplex::k_ring(op.mesh(), simplex::Simplex::vertex(op.mesh(), v), 1)
                         .simplex_vector(PrimitiveType::Vertex)) {
                    if (current_color ==
                        color_accessor.const_scalar_attribute(v_one_ring.tuple())) {
                        std::cout << "adjacent vertices have same color!!!" << std::endl;
                    }
                }
            }
        }
    }

    logger().debug("Executing on {} simplices", tups.size());

    {
        POLYSOLVE_SCOPED_STOPWATCH("Sorting", res.sorting_time, logger());

        // do sth or nothing
    }


    {
        POLYSOLVE_SCOPED_STOPWATCH("Executing operation", res.executing_time, logger());

        std::atomic_int suc_cnt = 0;
        std::atomic_int fail_cnt = 0;

        for (int64_t i = 0; i < colored_simplices.size(); ++i) {
            tbb::parallel_for(
                tbb::blocked_range<int64_t>(0, colored_simplices[i].size()),
                [&](tbb::blocked_range<int64_t> r) {
                    for (int64_t k = r.begin(); k < r.end(); ++k) {
                        auto mods = op(colored_simplices[i][k]);
                        if (mods.empty()) {
                            fail_cnt++;
                        } else {
                            suc_cnt++;
                        }
                    }
                });

            // for (int64_t k = 0; k < colored_simplices[i].size(); ++k) {
            //     auto mods = op(colored_simplices[i][k]);
            //     if (mods.empty()) {
            //         fail_cnt++;
            //     } else {
            //         suc_cnt++;
            //     }
            // }
        }

        res.m_num_op_success = suc_cnt;
        res.m_num_op_fail = fail_cnt;
    }

    // logger().debug(
    //     "Ran {} ops, {} succeeded, {} failed",
    //     res.number_of_performed_operations(),
    //     res.number_of_successful_operations(),
    //     res.number_of_failed_operations());

    m_stats += res;

    return res;
}

} // namespace wmtk
