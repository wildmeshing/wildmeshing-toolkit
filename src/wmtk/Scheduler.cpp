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
    op.reserve_enough_simplices();

    const auto type = op.primitive_type();
    {
        POLYSOLVE_SCOPED_STOPWATCH("Collecting primitives", res.collecting_time, logger());

        const auto tups = op.mesh().get_all(type);
        simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tups, type);
    }


    // logger().debug("Executing on {} simplices", simplices.size());

    {
        POLYSOLVE_SCOPED_STOPWATCH("Sorting", res.sorting_time, logger());
        if (op.use_random_priority()) {
            std::mt19937 gen(utils::get_random_seed());

            std::shuffle(simplices.begin(), simplices.end(), gen);
        } else {
            std::stable_sort(
                simplices.begin(),
                simplices.end(),
                [&op](const auto& s_a, const auto& s_b) {
                    return op.priority(s_a) < op.priority(s_b);
                });
        }
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("Executing operation", res.executing_time, logger());
        for (const simplex::Simplex& s : simplices) {
            auto mods = op(s);
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
    for (const auto& s : simplices) {
        flag_accessor.scalar_attribute(s.tuple()) = 0;
    }

    SchedulerStats res = run_operation_on_all(op);
    int64_t success = res.number_of_successful_operations();

    while (success > 0) {
        SchedulerStats internal_stats;
        op.reserve_enough_simplices();

        {
            POLYSOLVE_SCOPED_STOPWATCH(
                "Collecting primitives",
                internal_stats.collecting_time,
                logger());

            auto tups = op.mesh().get_all(type);
            tups.erase(
                std::remove_if(
                    tups.begin(),
                    tups.end(),
                    [&](const Tuple& t) { return flag_accessor.scalar_attribute(t) == 0; }),
                tups.end());
            simplices =
                wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tups, type);
        }

        {
            POLYSOLVE_SCOPED_STOPWATCH("Sorting", internal_stats.sorting_time, logger());
            if (op.use_random_priority()) {
                std::mt19937 gen(utils::get_random_seed());

                std::shuffle(simplices.begin(), simplices.end(), gen);
            } else {
                std::stable_sort(
                    simplices.begin(),
                    simplices.end(),
                    [&op](const auto& s_a, const auto& s_b) {
                        return op.priority(s_a) < op.priority(s_b);
                    });
            }
        }

        {
            POLYSOLVE_SCOPED_STOPWATCH(
                "Executing operation",
                internal_stats.executing_time,
                logger());
            for (const simplex::Simplex& s : simplices) {
                auto mods = op(s);
                if (mods.empty())
                    internal_stats.fail();
                else
                    internal_stats.succeed();
            }
        }

        success = internal_stats.number_of_successful_operations();
        res += internal_stats;
        m_stats += internal_stats;
    }

    return res;
}

} // namespace wmtk
