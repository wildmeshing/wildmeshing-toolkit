#include "Scheduler.hpp"

#include "Mesh.hpp"

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/random_seed.hpp>

#include <polysolve/Utils.hpp>


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
        simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tups, type);
    }


    // logger().debug("Executing on {} simplices", simplices.size());
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
        for (const auto& o : order) {
            auto mods = op(simplices[o.first]);
            if (mods.empty())
                res.fail();
            else
                res.succeed();
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

            simplices =
                wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tups, type);
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
            for (const auto& o : order) {
                auto mods = op(simplices[o.first]);
                if (mods.empty())
                    internal_stats.fail();
                else
                    internal_stats.succeed();
            }
        }

        success = internal_stats.number_of_successful_operations();
        res += internal_stats;
        res.sub_stats.push_back(internal_stats);
        m_stats += internal_stats;
        m_stats.sub_stats.push_back(internal_stats);
    }

    return res;
}

} // namespace wmtk
