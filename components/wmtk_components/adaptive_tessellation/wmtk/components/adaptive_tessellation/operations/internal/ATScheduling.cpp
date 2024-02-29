#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperations.hpp>

namespace wmtk::components::operations::internal {
std::vector<wmtk::simplex::Simplex>
ATOperations::get_all_edges_of_all_triangles_with_triangle_filter(
    std::shared_ptr<wmtk::Mesh> uv_mesh_ptr,
    wmtk::attribute::Accessor<double>& face_attr_accessor,
    double face_attr_filter_threshold)
{
    const auto tups = uv_mesh_ptr->get_all(PrimitiveType::Triangle);
    std::vector<wmtk::simplex::Simplex> edge_simplices;
    for (const auto& tup : tups) {
        double face_attr = face_attr_accessor.scalar_attribute(tup);
        if (face_attr < face_attr_filter_threshold) {
            continue;
        }
        // get all the edges simplices of each triangle tuple
        // current edge
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, tup));
        auto next_edge = uv_mesh_ptr->switch_tuple(tup, PrimitiveType::Edge);
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_edge));
        auto next_next_edge =
            uv_mesh_ptr->switch_tuples(tup, {PrimitiveType::Vertex, PrimitiveType::Edge});
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_next_edge));
    }
    return edge_simplices;
}
wmtk::SchedulerStats ATOperations::run_operation_on_top_of_given_simplices(
    std::vector<wmtk::simplex::Simplex>& edge_simplices,
    wmtk::operations::Operation& edge_op,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority)
{
    wmtk::SchedulerStats res;
    if (edge_simplices.empty()) {
        res.fail();
        return res;
    }
    std::stable_sort(
        edge_simplices.begin(),
        edge_simplices.end(),
        [&edge_priority](const auto& s_a, const auto& s_b) {
            return edge_priority(s_a) < edge_priority(s_b);
        });
    assert(edge_simplices.size() > 0);

    auto mods = edge_op(edge_simplices[0]);
    if (mods.empty())
        res.fail();
    else
        res.succeed();
    return res;
}
wmtk::SchedulerStats ATOperations::run_operation_on_all_given_simplices(
    std::vector<wmtk::simplex::Simplex>& edge_simplices,
    wmtk::operations::Operation& edge_op,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority)
{
    wmtk::SchedulerStats res;
    if (edge_simplices.empty()) {
        res.fail();
        return res;
    }
    std::stable_sort(
        edge_simplices.begin(),
        edge_simplices.end(),
        [&edge_priority](const auto& s_a, const auto& s_b) {
            return edge_priority(s_a) < edge_priority(s_b);
        });
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
} // namespace wmtk::components::operations::internal