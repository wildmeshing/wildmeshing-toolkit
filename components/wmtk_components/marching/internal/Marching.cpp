#include "Marching.hpp"

#include <deque>
#include <optional>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>

namespace wmtk::components::internal {

class TagAttribute
{
public:
    Accessor<int64_t> m_accessor;
    PrimitiveType m_ptype;
    int64_t m_val;

    TagAttribute(
        Mesh& m,
        const attribute::TypedAttributeHandle<int64_t>& attribute,
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
    std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t, int64_t>& vertex_tags,
    std::tuple<std::string, int64_t>& output_vertex_tag,
    std::vector<std::tuple<attribute::TypedAttributeHandle<int64_t>, int64_t>>& filter_tag)
    : m_mesh(mesh)
    , m_vertex_tags(vertex_tags)
    , m_output_vertex_tag(output_vertex_tag)
    , m_edge_filter_tags(filter_tag)
{
    m_pos_attribute =
        m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex).as<double>();
}

void Marching::process()
{
    using namespace operations;


    auto todo_attribute =
        m_mesh
            .register_attribute<int64_t>("todo_edgesplit_same_handle", wmtk::PrimitiveType::Edge, 1)
            .as<int64_t>();


    auto [vertex_tag_handle, vertex_tag_0, vertex_tag_1] = m_vertex_tags;
    Accessor<int64_t> acc_vertex_tag = m_mesh.create_accessor(vertex_tag_handle);

    std::deque<TagAttribute> filters;
    for (const auto& [edge_filter_handle, edge_filter_tag_value] : m_edge_filter_tags) {
        filters
            .emplace_back(m_mesh, edge_filter_handle, PrimitiveType::Edge, edge_filter_tag_value);
    }


    const auto& [output_name, output_value] = m_output_vertex_tag;

    auto output_tag_handle =
        m_mesh.get_attribute_handle<int64_t>(output_name, PrimitiveType::Vertex).as<int64_t>();

    TagAttribute output_accessor(m_mesh, output_tag_handle, PrimitiveType::Vertex, output_value);

    // compute the todo list for the split edge
    Accessor<int64_t> acc_todo = m_mesh.create_accessor(todo_attribute);
    for (const Tuple& edge : m_mesh.get_all(PrimitiveType::Edge)) {
        bool is_of_interest = true;
        for (const TagAttribute& filter : filters) {
            if (filter.m_accessor.const_scalar_attribute(edge) != filter.m_val) {
                is_of_interest = false;
                break;
            }
        }

        if (!is_of_interest) {
            continue;
        }

        const int64_t vt0 = acc_vertex_tag.scalar_attribute(edge);
        const int64_t vt1 = acc_vertex_tag.scalar_attribute(m_mesh.switch_vertex(edge));
        if ((vt0 == vertex_tag_0 && vt1 == vertex_tag_1) ||
            (vt1 == vertex_tag_0 && vt0 == vertex_tag_1)) {
            acc_todo.scalar_attribute(edge) = 1;
        }
    }

    EdgeSplit op_split(m_mesh);
    op_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_attribute));

    op_split.set_new_attribute_strategy(
        attribute::MeshAttributeHandle(m_mesh, *m_pos_attribute),
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);
    // vertex_tag_handle
    op_split.set_new_attribute_strategy(
        attribute::MeshAttributeHandle(m_mesh, vertex_tag_handle),
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::None);
    // output_tag_handle
    {
        const int64_t val = output_value;

        auto tmp = std::make_shared<SplitNewAttributeStrategy<int64_t>>(
            attribute::MeshAttributeHandle(m_mesh, output_tag_handle));
        tmp->set_strategy(SplitBasicStrategy::None);
        tmp->set_rib_strategy(
            [val](const VectorX<int64_t>&, const VectorX<int64_t>&, const std::bitset<2>&) {
                VectorX<int64_t> ret(1);
                ret(0) = val;
                return ret;
            });
        op_split.set_new_attribute_strategy(
            attribute::MeshAttributeHandle(m_mesh, output_tag_handle),
            tmp);
    }
    // filters
    for (const auto& [edge_filter_handle, _] : m_edge_filter_tags) {
        op_split.set_new_attribute_strategy(
            attribute::MeshAttributeHandle(m_mesh, edge_filter_handle),
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
    }
    // todo_attribute
    op_split.set_new_attribute_strategy(
        attribute::MeshAttributeHandle(m_mesh, todo_attribute),
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::None);


    Scheduler scheduler;
    while (true) {
        const auto stats = scheduler.run_operation_on_all(op_split);
        if (stats.number_of_successful_operations() == 0) {
            break;
        }
    }

    m_mesh.clear_attributes({vertex_tag_handle, output_tag_handle, *m_pos_attribute});
}

} // namespace wmtk::components::internal
