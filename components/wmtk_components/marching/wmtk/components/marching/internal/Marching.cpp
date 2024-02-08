#include "Marching.hpp"

#include <deque>
#include <optional>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

namespace wmtk::components::internal {

class TagAttribute
{
public:
    Accessor<int64_t> m_accessor;
    PrimitiveType m_ptype;
    int64_t m_val;

    TagAttribute(
        Mesh& m,
        const attribute::MeshAttributeHandle& attribute,
        PrimitiveType ptype,
        int64_t val)
        : m_accessor(m.create_accessor<int64_t>(attribute))
        , m_ptype(ptype)
        , m_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;
};


Marching::Marching(
    Mesh& mesh,
    attribute::MeshAttributeHandle& vertex_label,
    const std::vector<int64_t>& input_values,
    const int64_t output_value,
    std::vector<attribute::MeshAttributeHandle>& filter_labels,
    const std::vector<int64_t>& filter_values,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_vertex_label(vertex_label)
    , m_input_values(input_values)
    , m_output_value(output_value)
    , m_filter_labels(filter_labels)
    , m_filter_values(filter_values)
    , m_pass_through_attributes(pass_through_attributes)
{}

void Marching::process()
{
    using namespace operations;

    auto todo_attribute = m_mesh.register_attribute_typed<int64_t>(
        "todo_edgesplit_in_marching_component",
        wmtk::PrimitiveType::Edge,
        1);


    assert(m_input_values.size() == 2);
    const int64_t vertex_tag_0 = m_input_values[0];
    const int64_t vertex_tag_1 = m_input_values[1];
    Accessor<int64_t> acc_vertex_tag = m_mesh.create_accessor<int64_t>(m_vertex_label);

    std::deque<TagAttribute> filters;
    for (size_t i = 0; i < m_filter_labels.size(); ++i) {
        filters.emplace_back(m_mesh, m_filter_labels[i], PrimitiveType::Edge, m_filter_values[i]);
    }

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

    // vertex_label
    {
        /**************************edge tag******************************/
        // auto compute_edge_label = [this](const Eigen::MatrixXi& labels) -> Eigen::VectorXi {
        //     assert(labels.cols() == 2);
        //     if (labels(0, 0) == m_output_value && labels(1, 0) == m_output_value)
        //         return Eigen::VectorXi::Constant(1, m_output_value);
        //     return Eigen::VectorXi::Constant(1, 0);
        // };

        // // get edge_handle
        // auto edge_tag_handle = m_mesh.register_attribute_typed<int64_t>(
        //     "edge_tag_handle",
        //     wmtk::PrimitiveType::Edge,
        //     1);

        // std::shared_ptr etag_strategy =
        //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<int64_t,
        //     int64_t>>(
        //         edge_tag_handle,
        //         m_vertex_label,
        //         compute_edge_label);

        // op_split.add_transfer_strategy(etag_strategy);

        // /**************************face tag******************************/
        // auto compute_face_label = [this](const Eigen::MatrixXi& labels) -> Eigen::VectorXi {
        //     assert(labels.cols() == 3);
        //     if (labels(0, 0) == m_output_value && labels(0, 0) == m_output_value &&
        //         labels(0, 0) == m_output_value)
        //         return Eigen::VectorXi::Constant(1, m_output_value);
        //     return Eigen::VectorXi::Constant(1, 0);
        // };

        // // get face_handle
        // auto face_tag_handle = m_mesh.register_attribute_typed<int64_t>(
        //     "face_tag_handle",
        //     wmtk::PrimitiveType::Face,
        //     1);

        // std::shared_ptr ftag_strategy =
        //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<int64_t,
        //     int64_t>>(
        //         face_tag_handle,
        //         edge_tag_handle,
        //         compute_face_label);

        // op_split.add_transfer_strategy(ftag_strategy);

        auto tmp = std::make_shared<SplitNewAttributeStrategy<int64_t>>(m_vertex_label);
        tmp->set_strategy(SplitBasicStrategy::None);
        tmp->set_rib_strategy(
            [this](const VectorX<int64_t>&, const VectorX<int64_t>&, const std::bitset<2>&) {
                VectorX<int64_t> ret(1);
                ret(0) = m_output_value;
                return ret;
            });
        op_split.set_new_attribute_strategy(m_vertex_label, tmp);
    }

    // filters
    for (const auto& edge_filter_handle : m_filter_labels) {
        op_split.set_new_attribute_strategy(
            edge_filter_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
    }
    // todo_attribute
    op_split.set_new_attribute_strategy(
        attribute::MeshAttributeHandle(m_mesh, todo_attribute),
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::None);
    // pass_through
    for (const auto& attr : m_pass_through_attributes) {
        op_split.set_new_attribute_strategy(attr);
    }


    Scheduler scheduler;
    while (true) {
        const auto stats = scheduler.run_operation_on_all(op_split);
        if (stats.number_of_successful_operations() == 0) {
            break;
        }
    }
}


} // namespace wmtk::components::internal
