#pragma once

#include <spdlog/common.h>
#include <wmtk/operations/Operation.hpp>

namespace wmtk {

class SchedulerStats
{
public:
    /**
     * @brief Returns the number of successful operations performed by the scheduler.
     *
     * The value is reset to 0 when calling `run_operation_on_all`.
     */
    int64_t number_of_successful_operations() const { return m_num_op_success; }

    /**
     * @brief Returns the number of failed operations performed by the scheduler.
     *
     * The value is reset to 0 when calling `run_operation_on_all`.
     */
    int64_t number_of_failed_operations() const { return m_num_op_fail; }

    /**
     * @brief Returns the number of performed operations performed by the scheduler.
     *
     * The value is reset to 0 when calling `run_operation_on_all`.
     */
    int64_t number_of_performed_operations() const { return m_num_op_success + m_num_op_fail; }

    inline double total_time() const { return collecting_time + sorting_time + executing_time; }

    inline void succeed() { ++m_num_op_success; }
    inline void fail() { ++m_num_op_fail; }

    inline void operator+=(const SchedulerStats& s)
    {
        m_num_op_success += s.m_num_op_success;
        m_num_op_fail += s.m_num_op_fail;

        collecting_time += s.collecting_time;
        sorting_time += s.sorting_time;
        executing_time += s.executing_time;
    }


    double collecting_time = 0;
    double sorting_time = 0;
    double executing_time = 0;

    std::vector<SchedulerStats> sub_stats;

    double avg_sub_collecting_time() const
    {
        double res = 0;
        for (const auto& s : sub_stats) {
            res += s.collecting_time;
        }
        return res / sub_stats.size();
    }

    double avg_sub_sorting_time() const
    {
        double res = 0;
        for (const auto& s : sub_stats) {
            res += s.sorting_time;
        }
        return res / sub_stats.size();
    }

    double avg_sub_executing_time() const
    {
        double res = 0;
        for (const auto& s : sub_stats) {
            res += s.executing_time;
        }
        return res / sub_stats.size();
    }

    // private:
    int64_t m_num_op_success = 0;
    int64_t m_num_op_fail = 0;

    void print_update_log(size_t total, spdlog::level::level_enum = spdlog::level::info) const;
};

class Scheduler
{
public:
    Scheduler();
    ~Scheduler();

    SchedulerStats run_operation_on_all(operations::Operation& op);
    SchedulerStats run_operation_on_all(
        operations::Operation& op,
        const TypedAttributeHandle<char>& flag_handle);
    SchedulerStats run_operation_on_all_coloring(
        operations::Operation& op,
        const TypedAttributeHandle<int64_t>& color_handle);

    const SchedulerStats& stats() const { return m_stats; }

    void set_update_frequency(std::optional<size_t>&& freq = {});

private:
    SchedulerStats m_stats;
    std::optional<size_t> m_update_frequency = {};

    void log(const size_t total);
    void log(const SchedulerStats& stats, const size_t total);
};

} // namespace wmtk
