#include "Marching.hpp"

#include <optional>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>

namespace wmtk::components::internal {

class TagAttribute
{
public:
    Accessor<int64_t> m_accessor;
    PrimitiveType m_ptype;
    int64_t m_val;

    TagAttribute(
        Mesh& m,
        const MeshAttributeHandle<int64_t>& attribute,
        PrimitiveType ptype,
        int64_t val)
        : m_accessor(m.create_accessor(attribute))
        , m_ptype(ptype)
        , m_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;
};


Marching::Marching(
    Mesh& mesh,
    std::tuple<MeshAttributeHandle<int64_t>, int64_t, int64_t>& vertex_tags,
    std::tuple<std::string, int64_t>& output_vertex_tag,
    std::vector<std::tuple<MeshAttributeHandle<int64_t>, int64_t>>& filter_tag)
    : m_mesh(mesh)
    , m_vertex_tags(vertex_tags)
    , m_output_vertex_tag(output_vertex_tag)
    , m_edge_filter_tags(filter_tag)
{}

void Marching::process()
{
    using namespace operations;

    //// remove strategies and build new ones
    // m_mesh.m_split_strategies.clear();
    // m_mesh.m_collapse_strategies.clear();
    //
    //// set split position to mean
    //{
    //    m_pos_attribute = std::make_unique<attribute::AttributeInitializationHandle<double>>(
    //        m_mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3, true));
    //    m_pos_attribute->trimesh_standard_split_strategy().set_standard_split_rib_strategy(
    //        operations::NewAttributeStrategy::SplitRibBasicStrategy::Mean);
    //}
    //
    //
    // attribute::AttributeInitializationHandle<int64_t> todo_attribute =
    //    m_mesh.register_attribute<int64_t>("todo_edgesplit_same_handle",
    //    wmtk::PrimitiveType::Edge, 1);
    //
    //// todo attribute is set to default value after splt
    // todo_attribute.trimesh_standard_split_strategy().set_standard_split_strategy(
    //     operations::NewAttributeStrategy::SplitBasicStrategy::None);
    // todo_attribute.trimesh_standard_split_strategy().set_standard_split_rib_strategy(
    //     operations::NewAttributeStrategy::SplitRibBasicStrategy::None);
    //
    //
    // auto [vertex_tag_handle, vertex_tag_0, vertex_tag_1] = m_vertex_tags;
    // Accessor<int64_t> acc_vertex_tag = m_mesh.create_accessor(vertex_tag_handle);
    //
    // std::deque<TagAttribute> filters;
    // for (const auto& [edge_filter_handle, edge_filter_tag_value] : m_edge_filter_tags) {
    //     filters
    //         .emplace_back(m_mesh, edge_filter_handle, PrimitiveType::Edge,
    //         edge_filter_tag_value);
    // }
    //
    //
    // const auto& [output_name, output_value] = m_output_vertex_tag;
    //
    // attribute::AttributeInitializationHandle<int64_t> output_tag_handle =
    //     m_mesh.register_attribute<int64_t>(output_name, PrimitiveType::Vertex, 1, true);
    //
    // TagAttribute output_accessor(
    //     m_mesh,
    //     output_tag_handle,
    //     output_tag_handle.primitive_type(),
    //     output_value);
    //
    //{
    //     const int64_t val = output_value;
    //
    //     switch (m_mesh.top_simplex_type()) {
    //     case PrimitiveType::Face: {
    //         auto& strat = output_tag_handle.trimesh_standard_split_strategy();
    //         strat.set_split_rib_strategy([val](const VectorX<int64_t>&, const VectorX<int64_t>&)
    //         {
    //             VectorX<int64_t> ret(1);
    //             ret(0) = val;
    //             return ret;
    //         });
    //         break;
    //     }
    //     case PrimitiveType::Tetrahedron: {
    //         throw std::runtime_error("Implementation for tetrahedra meshes is incomplete");
    //        // auto& strat = output_tag_handle.tetmesh_standard_split_strategy();
    //        // strat.set_split_rib_strategy([val](const VectorX<int64_t>&, const VectorX<int64_t>&) {
    //        //     VectorX<int64_t> ret(1);
    //        //     ret(0) = val;
    //        //     return ret;
    //        // });
    //        // break;
    //    }
    //    default:
    //        throw std::runtime_error(
    //            "Marching is only implemented for triangle and tetrahedra meshes.");
    //    }
    //}
    //
    //
    //// edge split
    // Scheduler scheduler;
    //{
    //    // compute the todo list for the split edge
    //    Accessor<int64_t> acc_todo = m_mesh.create_accessor(todo_attribute);
    //    for (const Tuple& edge : m_mesh.get_all(PrimitiveType::Edge)) {
    //        bool is_of_interest = true;
    //        for (const TagAttribute& filter : filters) {
    //            if (filter.m_accessor.const_scalar_attribute(edge) != filter.m_val) {
    //                is_of_interest = false;
    //                break;
    //            }
    //        }
    //
    //        if (!is_of_interest) {
    //            continue;
    //        }
    //
    //        const int64_t vt0 = acc_vertex_tag.scalar_attribute(edge);
    //        const int64_t vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
    //        if ((vt0 == vertex_tag_0 && vt1 == vertex_tag_1) ||
    //            (vt1 == vertex_tag_0 && vt0 == vertex_tag_1)) {
    //            acc_todo.scalar_attribute(edge) = 1;
    //        }
    //    }
    //}
    //
    // EdgeSplit op_split(m_mesh);
    // op_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_attribute));
    // while (true) {
    //    auto stats = scheduler.run_operation_on_all(op_split);
    //    if (stats.number_of_successful_operations() == 0) {
    //        break;
    //    }
    //}
}

} // namespace wmtk::components::internal
