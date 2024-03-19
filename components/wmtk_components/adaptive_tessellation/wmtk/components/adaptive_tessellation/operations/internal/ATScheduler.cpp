#include "ATScheduler.hpp"
#include <wmtk/components/adaptive_tessellation/invariants/RGBSplitInvariant.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>
#include <wmtk/components/adaptive_tessellation/operations/utils/collect_todo_simplices.hpp>
#include <wmtk/components/adaptive_tessellation/operations/utils/tag_todo_edges.hpp>
namespace wmtk {

wmtk::SchedulerStats ATScheduler::run_operation_on_first_of_given_simplices(
    std::vector<wmtk::simplex::Simplex>& edge_simplices,
    wmtk::operations::Operation& edge_op,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority)
{
    wmtk::SchedulerStats res;
    if (edge_simplices.empty()) {
        res.fail();
        return res;
    }
    if (edge_priority != nullptr) {
        std::stable_sort(
            edge_simplices.begin(),
            edge_simplices.end(),
            [&edge_priority](const auto& s_a, const auto& s_b) {
                return edge_priority(s_a) < edge_priority(s_b);
            });
    }
    assert(edge_simplices.size() > 0);

    auto mods = edge_op(edge_simplices[0]);
    if (mods.empty())
        res.fail();
    else
        res.succeed();
    return res;
}

wmtk::SchedulerStats ATScheduler::run_operation_on_all_given_simplices(
    std::vector<wmtk::simplex::Simplex>& edge_simplices,
    wmtk::operations::Operation& edge_op,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority)
{
    wmtk::SchedulerStats res;
    if (edge_simplices.empty()) {
        res.fail();
        return res;
    }
    if (edge_priority != nullptr) {
        std::stable_sort(
            edge_simplices.begin(),
            edge_simplices.end(),
            [&edge_priority](const auto& s_a, const auto& s_b) {
                return edge_priority(s_a) < edge_priority(s_b);
            });
    }
    assert(edge_simplices.size() > 0);

    for (const wmtk::simplex::Simplex& s : edge_simplices) {
        auto mods = edge_op(s);
        if (mods.empty())
            res.fail();
        else
            res.succeed();
    }
    logger().info(
        "Ran {} ops, {} succeeded, {} failed",
        res.number_of_performed_operations(),
        res.number_of_successful_operations(),
        res.number_of_failed_operations());
    return res;
}

void ATScheduler::rgb_split_and_swap(
    std::shared_ptr<wmtk::Mesh>& uv_mesh_ptr,
    wmtk::attribute::Accessor<double>& triangle_distance_accessor,
    wmtk::operations::Operation& split,
    wmtk::operations::Operation& swap,
    double target_distance,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)> split_priority,
    bool one_operation_per_pass)
{
    int64_t split_success = 0;
    int64_t swap_success = 0;
    int64_t cnt = 0;
    do {
        logger().info("Pass {}", cnt);
        std::vector<wmtk::simplex::Simplex> all_edge_of_all_triangles = wmtk::components::
            operations::utils::get_all_edges_of_all_triangles_with_triangle_filter(
                uv_mesh_ptr,
                triangle_distance_accessor,
                target_distance);
        if (one_operation_per_pass) {
            split_success = run_operation_on_first_of_given_simplices(
                                all_edge_of_all_triangles,
                                split,
                                split_priority)
                                .number_of_successful_operations();

        } else {
            split_success = run_operation_on_all_given_simplices(
                                all_edge_of_all_triangles,
                                split,
                                split_priority)
                                .number_of_successful_operations();
        }
        do {
            swap_success = run_operation_on_all(swap).number_of_successful_operations();
        } while (swap_success > 0);

        cnt++;
    } while (split_success > 0);
}
void ATScheduler::rgb_recursive_split_swap(
    std::shared_ptr<wmtk::Mesh>& uv_mesh_ptr,
    wmtk::attribute::MeshAttributeHandle& face_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& edge_rgb_state_handle,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    wmtk::operations::Operation& split,
    wmtk::operations::Operation& swap)
{
    const attribute::Accessor<int64_t> face_rgb_state_accessor =
        uv_mesh_ptr->create_const_accessor(face_rgb_state_handle.as<int64_t>());
    const attribute::Accessor<int64_t> edge_rgb_state_accessor =
        uv_mesh_ptr->create_const_accessor(edge_rgb_state_handle.as<int64_t>());
    while (true) {
        while (true) {
            const auto stats = run_operation_on_all(split);
            if (stats.number_of_successful_operations() == 0) {
                break;
            }
        }
        while (true) {
            const auto stats = run_operation_on_all(swap);
            if (stats.number_of_successful_operations() == 0) {
                break;
            }
        }
        int64_t todo_edge_cnt = 0;
        for (auto& e : uv_mesh_ptr->get_all(wmtk::PrimitiveType::Edge)) {
            if (edge_todo_accessor.scalar_attribute(e) == 1) {
                wmtk::components::operations::utils::tag_secondary_split_edges(
                    uv_mesh_ptr,
                    face_rgb_state_accessor,
                    edge_rgb_state_accessor,
                    edge_todo_accessor,
                    e);
                todo_edge_cnt++;
            }
        }
        if (todo_edge_cnt == 0) {
            break;
        }
    }
}

void ATScheduler::rgb_scheduling(
    std::shared_ptr<wmtk::Mesh>& uv_mesh_ptr,
    wmtk::attribute::MeshAttributeHandle& face_rgb_state_handle,
    wmtk::attribute::MeshAttributeHandle& edge_rgb_state_handle,
    wmtk::attribute::Accessor<int64_t>& edge_todo_accessor,
    wmtk::operations::Operation& split,
    wmtk::operations::Operation& swap,
    wmtk::attribute::Accessor<double>& triangle_distance_accessor,
    wmtk::attribute::Accessor<double>& curved_edge_length_accessor,
    double target_distance)
{
    attribute::Accessor<int64_t> face_rgb_state_accessor =
        uv_mesh_ptr->create_const_accessor(face_rgb_state_handle.as<int64_t>());
    attribute::Accessor<int64_t> edge_rgb_state_accessor =
        uv_mesh_ptr->create_const_accessor(edge_rgb_state_handle.as<int64_t>());

    while (true) {
        int64_t todo_edge_cnt = wmtk::components::operations::utils::tag_longest_edge_of_all_faces(
            uv_mesh_ptr,
            edge_todo_accessor,
            triangle_distance_accessor,
            curved_edge_length_accessor,
            target_distance);

        if (todo_edge_cnt == 0) {
            break;
        }
        rgb_recursive_split_swap(
            uv_mesh_ptr,
            face_rgb_state_handle,
            edge_rgb_state_handle,
            edge_todo_accessor,
            split,
            swap);
    }
}
} // namespace wmtk