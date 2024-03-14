#include <wmtk/Scheduler.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk {
class ATScheduler : public Scheduler
{
public:
    wmtk::SchedulerStats run_operation_on_all_given_simplices(
        std::vector<wmtk::simplex::Simplex>& edge_simplices,
        wmtk::operations::Operation& edge_op,
        std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority);

    wmtk::SchedulerStats run_operation_on_first_of_given_simplices(
        std::vector<wmtk::simplex::Simplex>& edge_simplices,
        wmtk::operations::Operation& edge_op,
        std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority);

    void rgb_split_and_swap(
        std::shared_ptr<wmtk::Mesh>& uv_mesh_ptr,
        wmtk::attribute::Accessor<double>& triangle_distance_accessor,
        wmtk::operations::Operation& split,
        wmtk::operations::Operation& swap,
        double target_distance,
        std::function<std::vector<double>(const simplex::Simplex&)> split_priority = nullptr,
        bool one_operation_per_pass = false);
    void rgb_split_scheduling(
        std::shared_ptr<wmtk::Mesh>& uv_mesh_ptr,
        wmtk::attribute::Accessor<double>& edge_todo_accessor,
        wmtk::operations::Operation& split);

protected:
    SchedulerStats m_at_stats;
};
} // namespace wmtk