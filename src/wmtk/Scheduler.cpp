#include "Scheduler.hpp"

#include "Mesh.hpp"

#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>

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

    const auto type = op.primitive_type();
    {
        POLYSOLVE_SCOPED_STOPWATCH("Collecting primitives", res.collecting_time, logger());

        const auto tups = op.mesh().get_all(type);
        simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(tups, type);
    }


    logger().info("Executing on {} simplices", simplices.size());

    {
        POLYSOLVE_SCOPED_STOPWATCH("Sorting", res.sorting_time, logger());
        if (op.use_random_priority()) {
            std::random_device rd;
            std::mt19937 g(rd());

            std::shuffle(simplices.begin(), simplices.end(), g);
        } else {
            std::sort(simplices.begin(), simplices.end(), [&op](const auto& s_a, const auto& s_b) {
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

    logger().debug(
        "Ran {} ops, {} succeeded, {} failed",
        res.number_of_performed_operations(),
        res.number_of_successful_operations(),
        res.number_of_failed_operations());

    m_stats += res;

    return res;
}

} // namespace wmtk
