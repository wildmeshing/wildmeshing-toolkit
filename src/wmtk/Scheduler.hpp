#pragma once

#include "operations/Operation.hpp"

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

private:
    int64_t m_num_op_success = 0;
    int64_t m_num_op_fail = 0;
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

    const SchedulerStats& stats() const { return m_stats; }

private:
    SchedulerStats m_stats;
};

} // namespace wmtk
