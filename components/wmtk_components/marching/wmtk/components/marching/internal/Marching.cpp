#include "Marching.hpp"

#include <deque>
#include <optional>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>

namespace wmtk::components::internal {

class TagAttribute
{
public:
    wmtk::attribute::Accessor<int64_t> m_accessor;
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
    attribute::MeshAttributeHandle& vertex_tag_handle,
    const std::vector<int64_t>& input_values,
    const int64_t output_value,
    const double weight,
    std::vector<attribute::MeshAttributeHandle>& filter_labels,
    const std::vector<int64_t>& filter_values,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_input_values(input_values)
    , m_output_value(output_value)
    , m_filter_labels(filter_labels)
    , m_filter_values(filter_values)
    , m_pass_through_attributes(pass_through_attributes)
    , m_weight(weight)
{}

Marching::Marching(
    Mesh& mesh,
    attribute::MeshAttributeHandle& vertex_tag_handle,
    attribute::MeshAttributeHandle& edge_tag_handle,
    const std::vector<int64_t>& input_values,
    const int64_t output_value,
    const double weight,
    std::vector<attribute::MeshAttributeHandle>& filter_labels,
    const std::vector<int64_t>& filter_values,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_edge_tag_handle(edge_tag_handle)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_input_values(input_values)
    , m_output_value(output_value)
    , m_filter_labels(filter_labels)
    , m_filter_values(filter_values)
    , m_pass_through_attributes(pass_through_attributes)
    , m_weight(weight)
{}

Marching::Marching(
    Mesh& mesh,
    attribute::MeshAttributeHandle& vertex_tag_handle,
    attribute::MeshAttributeHandle& edge_tag_handle,
    attribute::MeshAttributeHandle& face_tag_handle,
    const std::vector<int64_t>& input_values,
    const int64_t output_value,
    const double weight,
    std::vector<attribute::MeshAttributeHandle>& filter_labels,
    const std::vector<int64_t>& filter_values,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_edge_tag_handle(edge_tag_handle)
    , m_face_tag_handle(face_tag_handle)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_input_values(input_values)
    , m_output_value(output_value)
    , m_filter_labels(filter_labels)
    , m_filter_values(filter_values)
    , m_pass_through_attributes(pass_through_attributes)
    , m_weight(weight)
{}

void Marching::process()
{
    using namespace operations;

    auto todo_attribute = m_mesh.register_attribute_typed<int64_t>(
        "todo_edgesplit_in_marching_component",
        wmtk::PrimitiveType::Edge,
        1);
    auto splitted_edges_attribute = m_mesh.register_attribute_typed<int64_t>(
        "splitted_edges_in_marching_component",
        wmtk::PrimitiveType::Edge,
        1);

    assert(m_input_values.size() == 2);
    const int64_t vertex_tag_0 = m_input_values[0];
    const int64_t vertex_tag_1 = m_input_values[1];
    wmtk::attribute::Accessor<int64_t> acc_vertex_tag =
        m_mesh.create_accessor<int64_t>(m_vertex_tag_handle);

    std::deque<TagAttribute> filters;
    for (size_t i = 0; i < m_filter_labels.size(); ++i) {
        filters.emplace_back(m_mesh, m_filter_labels[i], PrimitiveType::Edge, m_filter_values[i]);
    }

    // compute the todo list for the split edge
    wmtk::attribute::Accessor<int64_t> acc_todo = m_mesh.create_accessor(todo_attribute);
    wmtk::attribute::Accessor<int64_t> acc_splitted_edges =
        m_mesh.create_accessor(splitted_edges_attribute);
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
        const int64_t vt1 =
            acc_vertex_tag.scalar_attribute(m_mesh.switch_tuple(edge, PrimitiveType::Vertex));
        if ((vt0 == vertex_tag_0 && vt1 == vertex_tag_1) ||
            (vt1 == vertex_tag_0 && vt0 == vertex_tag_1)) {
            acc_todo.scalar_attribute(edge) = 1;
            acc_splitted_edges.scalar_attribute(edge) = 1;
        }
    }

    EdgeSplit op_split(m_mesh);
    op_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_attribute));

    // labels
    {
        /**************************edge tag******************************/
        if (m_edge_tag_handle.has_value()) {
            auto compute_edge_label =
                [this](const Eigen::MatrixX<int64_t>& labels) -> Eigen::VectorX<int64_t> {
                assert(labels.cols() == 2);
                if (labels(0, 0) == m_output_value && labels(0, 1) == m_output_value)
                    return Eigen::VectorX<int64_t>::Constant(1, m_output_value);
                return Eigen::VectorX<int64_t>::Constant(1, 0);
            };

            std::shared_ptr edge_tag_strategy = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
                m_edge_tag_handle.value(),
                m_vertex_tag_handle,
                compute_edge_label);

            op_split.add_transfer_strategy(edge_tag_strategy);
            op_split.set_new_attribute_strategy(m_edge_tag_handle.value());
        }

        /**************************face tag******************************/
        if (m_face_tag_handle.has_value() && m_edge_tag_handle.has_value()) {
            auto compute_face_label =
                [this](const Eigen::MatrixX<int64_t>& labels) -> Eigen::VectorX<int64_t> {
                assert(labels.cols() == 3);
                if (labels(0, 0) == m_output_value && labels(0, 1) == m_output_value &&
                    labels(0, 2) == m_output_value) {
                    return Eigen::VectorX<int64_t>::Constant(1, m_output_value);
                }
                return Eigen::VectorX<int64_t>::Constant(1, 0);
            };

            std::shared_ptr face_tag_strategy = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
                m_face_tag_handle.value(),
                m_edge_tag_handle.value(),
                compute_face_label);

            op_split.add_transfer_strategy(face_tag_strategy);
            op_split.set_new_attribute_strategy(m_face_tag_handle.value());
        }


        auto tmp = std::make_shared<SplitNewAttributeStrategy<int64_t>>(m_vertex_tag_handle);
        tmp->set_strategy(SplitBasicStrategy::None);
        tmp->set_rib_strategy(
            [this](const VectorX<int64_t>&, const VectorX<int64_t>&, const std::bitset<2>&) {
                VectorX<int64_t> ret(1);
                ret(0) = m_output_value;
                return ret;
            });
        op_split.set_new_attribute_strategy(m_vertex_tag_handle, tmp);

        op_split.set_new_attribute_strategy(
            attribute::MeshAttributeHandle(m_mesh, splitted_edges_attribute),
            SplitBasicStrategy::Copy,
            SplitRibBasicStrategy::None);
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

    // move vertices according to weight
    auto pos_handle = m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m_mesh.create_accessor<double>(pos_handle);
    for (const Tuple& v : m_mesh.get_all(PrimitiveType::Vertex)) {
        if (acc_vertex_tag.const_scalar_attribute(v) != m_output_value) {
            continue;
        }
        // get the two input positions
        Tuple v0;
        Tuple v1;
        auto incident_edges = simplex::cofaces_single_dimension_tuples(
            m_mesh,
            simplex::Simplex::vertex(m_mesh, v),
            PrimitiveType::Edge);
        for (Tuple e : incident_edges) {
            if (acc_splitted_edges.const_scalar_attribute(e) == 0) {
                continue;
            }
            if (acc_vertex_tag.const_scalar_attribute(e) == m_output_value) {
                e = m_mesh.switch_tuple(e, PrimitiveType::Vertex);
            }
            if (acc_vertex_tag.const_scalar_attribute(e) == vertex_tag_0) {
                v0 = e;
            }
            if (acc_vertex_tag.const_scalar_attribute(e) == vertex_tag_1) {
                v1 = e;
            }
        }
        assert(m_mesh.is_valid_with_hash(v0));
        assert(m_mesh.is_valid_with_hash(v1));
        const auto p0 = pos_acc.const_vector_attribute(v0);
        const auto p1 = pos_acc.const_vector_attribute(v1);
        pos_acc.vector_attribute(v) = (1 - m_weight) * p0 + m_weight * p1;
    }
}


} // namespace wmtk::components::internal
