#include "RGB2.hpp"
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/utils/Logger.hpp>
#include "invariants/TriMeshRGBSplitInvariant.hpp"
#include "invariants/TriMeshRGBSwapInvariant.hpp"
#include "operations/RGBEdgeSplit.hpp"
#include "operations/RGBEdgeSwap.hpp"
namespace wmtk::components::rgb {


void RGB2::register_attributes()
{
    m_edge_length_handle =
        mesh().register_attribute<int64_t>("edge_rgb_level", PrimitiveType::Edge, 2, true, 0);
    m_triangle_level_handle =
        mesh()
            .register_attribute<int64_t>("triangle_rgb_level", PrimitiveType::Triangle, 2, true, 0);
    m_edge_level_handle =
        mesh().register_attribute<double>("edge_length", PrimitiveType::Edge, 1, true, 0.0);
    auto edge_todo_handle =
        mesh().register_attribute<int64_t>("todo", PrimitiveType::Edge, 1, true, 0);
}


RGB2::~RGB2() = default;
TriMesh& RGB2::mesh()
{
    auto ptr = dynamic_cast<TriMesh*>(&const_cast<Mesh&>(position_handle().mesh()));
    assert(ptr != nullptr);
    return *ptr;
}
const TriMesh& RGB2::mesh() const
{
    auto ptr = dynamic_cast<const TriMesh*>(&position_handle().mesh());
    assert(ptr != nullptr);
    return *ptr;
}
RGB2::RGB2(const RGBOptions& opts)
    : RGB(opts)
{
    register_attributes();
}

void RGB2::create_split()
{
    // create operation
    m_split_op = std::make_shared<operations::RGBEdgeSplit>(
        mesh(),
        m_triangle_level_handle,
        m_edge_length_handle);

    // todo: update rules
    m_split_op->use_random_priority() = true;
    m_split_op->add_invariant(
        std::make_shared<wmtk::TodoInvariant>(mesh(), m_todo_handle.as<int64_t>(), 1));
    m_split_op->add_invariant(std::make_shared<invariants::TriMeshRGBSplitInvariant>(
        mesh(),
        m_triangle_level_handle.as<int64_t>(),
        m_edge_level_handle.as<int64_t>()));
}

void RGB2::create_swap()
{
    m_swap_op = std::make_shared<operations::RGBTriEdgeSwap>(
        mesh(),
        m_triangle_level_handle,
        m_edge_level_handle);

    // update attributes
    m_swap_op->use_random_priority() = true;
    m_swap_op->add_invariant(std::make_shared<invariants::TriMeshRGBSwapInvariant>(
        mesh(),
        m_triangle_level_handle.as<int64_t>(),
        m_edge_level_handle.as<int64_t>()));
    m_swap_op->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(mesh()));
    m_swap_op->add_invariant(std::make_shared<InteriorEdgeInvariant>(mesh()));
}

void RGB2::run()
{
    {
        // Initialize attributes and log the number of edges to process
        // TODO: set up TODO to do hte long edges


        wmtk::attribute::Accessor<int64_t> todo_accessor =
            mesh().create_accessor<int64_t>(m_todo_handle);

        Scheduler scheduler;

        while (true) {
            logger().info("egde num {}", mesh().get_all(PrimitiveType::Edge).size());
            // TODO: Tag the longest edges and check if there are edges to process

            // logger().info("todo_edge_cnt {}", todo_edge_cnt);
            // if (todo_edge_cnt == 0) {
            //    break;
            //}
            // multimesh::consolidate(mesh());
            while (true) {
                // Split loop
                while (true) {
                    const auto split_stats = scheduler.run_operation_on_all(*m_split_op);
                    if (split_stats.number_of_successful_operations() == 0) {
                        logger().warn("No successful split operations");
                        break;
                    }
                }
                // multimesh::consolidate(*parameter_mesh);

                // Swap loop
                while (true) {
                    const auto swap_stats = scheduler.run_operation_on_all(*m_swap_op);
                    if (swap_stats.number_of_successful_operations() == 0) {
                        break;
                    }
                    // multimesh::consolidate(*parameter_mesh);
                }

                // Tag secondary split edges and check for remaining edges
                int64_t inner_todo_edge_cnt = 0;
                for (auto& e : mesh().get_all(wmtk::PrimitiveType::Edge)) {
                    if (todo_accessor.scalar_attribute(e) == 1) {
                        // tag secondary split edges
                        inner_todo_edge_cnt++;
                    }
                }
                // logger().warn("Remaining edges to process: {}", inner_todo_edge_cnt);
                if (inner_todo_edge_cnt == 0) {
                    break;
                }
            }
        }
    }
}
} // namespace wmtk::components::rgb
