#include "Marching.hpp"

#include <deque>
#include <optional>
#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

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
    attribute::MeshAttributeHandle& pos_handle,
    std::map<PrimitiveType, attribute::MeshAttributeHandle>& label_handles,
    const std::vector<int64_t>& input_values,
    const int64_t output_value)
    : m_mesh(pos_handle.mesh())
    , m_pos_handle(pos_handle)
    , m_input_values(input_values)
    , m_output_value(output_value)
{
    assert(label_handles.count(PrimitiveType::Vertex));

    m_vertex_tag_handle = label_handles[PrimitiveType::Vertex];

    if (label_handles.count(PrimitiveType::Edge)) {
        m_edge_tag_handle = label_handles[PrimitiveType::Edge];
    }

    if (label_handles.count(PrimitiveType::Triangle)) {
        m_face_tag_handle = label_handles[PrimitiveType::Triangle];
    }
}

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

    assert(m_input_values.size() == 1 || m_input_values.size() == 2);

    if (m_input_values.size() == 1) {
        logger().info(
            "Only one input value was given. Perform marching in between value {} and any other "
            "value.",
            m_input_values[0]);
    }

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
            if ((filter.m_accessor.const_scalar_attribute(edge) == filter.m_val) ==
                m_invert_filter) {
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
        if (m_input_values.size() == 2) {
            if ((vt0 == m_input_values[0] && vt1 == m_input_values[1]) ||
                (vt1 == m_input_values[0] && vt0 == m_input_values[1])) {
                acc_todo.scalar_attribute(edge) = 1;
                acc_splitted_edges.scalar_attribute(edge) = 1;
            }
        } else {
            assert(m_input_values.size() == 1);
            if ((vt0 == m_input_values[0] && vt1 != m_input_values[0]) ||
                (vt1 == m_input_values[0] && vt0 != m_input_values[0])) {
                acc_todo.scalar_attribute(edge) = 1;
                acc_splitted_edges.scalar_attribute(edge) = 1;
            }
        }
    }

    EdgeSplit op_split(m_mesh);
    op_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_attribute));

    // position
    op_split.set_new_attribute_strategy(m_pos_handle);

    // labels
    {
        /**************************edge tag******************************/
        if (m_edge_tag_handle.has_value()) {
            auto compute_edge_label =
                [this](
                    const Eigen::MatrixX<int64_t>& labels,
                    const std::vector<Tuple>& tuples) -> Eigen::VectorX<int64_t> {
                assert(labels.cols() == 2);
                assert(labels.rows() == 1);
                assert(tuples.size() == 2);
                if (labels(0, 0) == m_output_value && labels(0, 1) == m_output_value) {
                    return Eigen::VectorX<int64_t>::Constant(1, m_output_value);
                }

                // do not change the current value
                auto acc = m_mesh.create_const_accessor<int64_t>(m_edge_tag_handle.value());
                const int64_t val = acc.const_scalar_attribute(tuples[0]);
                return Eigen::VectorX<int64_t>::Constant(1, val);
            };

            std::shared_ptr edge_tag_strategy = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
                m_edge_tag_handle.value(),
                m_vertex_tag_handle,
                compute_edge_label);

            op_split.set_new_attribute_strategy(
                m_edge_tag_handle.value(),
                SplitBasicStrategy::Copy,
                SplitRibBasicStrategy::None);
            op_split.add_transfer_strategy(edge_tag_strategy);
        }

        /**************************face tag******************************/
        if (m_face_tag_handle.has_value() && m_edge_tag_handle.has_value()) {
            auto compute_face_label =
                [this](
                    const Eigen::MatrixX<int64_t>& labels,
                    const std::vector<Tuple>& tuples) -> Eigen::VectorX<int64_t> {
                assert(labels.cols() == 3);
                assert(labels.rows() == 1);
                assert(tuples.size() == 3);
                if (labels(0, 0) == m_output_value && labels(0, 1) == m_output_value &&
                    labels(0, 2) == m_output_value) {
                    return Eigen::VectorX<int64_t>::Constant(1, m_output_value);
                }

                // do not change the current value
                auto acc = m_mesh.create_const_accessor<int64_t>(m_face_tag_handle.value());
                const int64_t val = acc.const_scalar_attribute(tuples[0]);
                return Eigen::VectorX<int64_t>::Constant(1, val);
            };

            std::shared_ptr face_tag_strategy = std::make_shared<
                wmtk::operations::SingleAttributeTransferStrategy<int64_t, int64_t>>(
                m_face_tag_handle.value(),
                m_edge_tag_handle.value(),
                compute_face_label);

            op_split.set_new_attribute_strategy(
                m_face_tag_handle.value(),
                SplitBasicStrategy::Copy,
                SplitRibBasicStrategy::None);
            op_split.add_transfer_strategy(face_tag_strategy);
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
    // scalar field
    if (m_scalar_field.has_value()) {
        op_split.set_new_attribute_strategy(
            m_scalar_field.value(),
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::None);
    }
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

    if (m_scalar_field.has_value()) {
        // move vertices according to isovalue
        auto sf_acc = m_mesh.create_accessor<double>(m_scalar_field.value());

        auto pos_acc = m_mesh.create_accessor<double>(m_pos_handle);

        auto inversion_invariant =
            SimplexInversionInvariant<double>(m_mesh, m_pos_handle.as<double>());

        for (const Tuple& v_tuple : m_mesh.get_all(PrimitiveType::Vertex)) {
            const simplex::Simplex v(m_mesh, PrimitiveType::Vertex, v_tuple);

            if (acc_vertex_tag.const_scalar_attribute(v) != m_output_value) {
                continue;
            }
            // get the two input positions
            Tuple v0;
            Tuple v1;
            auto incident_edges =
                simplex::cofaces_single_dimension_tuples(m_mesh, v, PrimitiveType::Edge);
            for (Tuple e : incident_edges) {
                if (acc_splitted_edges.const_scalar_attribute(e) == 0) {
                    continue;
                }
                if (acc_vertex_tag.const_scalar_attribute(e) == m_output_value) {
                    e = m_mesh.switch_tuple(e, PrimitiveType::Vertex);
                }
                if (acc_vertex_tag.const_scalar_attribute(e) == m_input_values[0]) {
                    v0 = e;
                }
                if (m_input_values.size() == 2) {
                    if (acc_vertex_tag.const_scalar_attribute(e) == m_input_values[1]) {
                        v1 = e;
                    }
                } else {
                    if (acc_vertex_tag.const_scalar_attribute(e) != m_input_values[0]) {
                        v1 = e;
                    }
                }
            }
            assert(m_mesh.is_valid(v0));
            assert(m_mesh.is_valid(v1));
            const auto p0 = pos_acc.const_vector_attribute(v0);
            const auto p1 = pos_acc.const_vector_attribute(v1);
            const double sf0 = sf_acc.const_scalar_attribute(v0);
            const double sf1 = sf_acc.const_scalar_attribute(v1);

            auto p = pos_acc.vector_attribute(v);

            const double u = (m_isovalue - sf0) / (sf1 - sf0);

            const Eigen::VectorXd p_opt = (1. - u) * p0 + u * p1;
            const Eigen::VectorXd p_mid = p;


            const auto tets = simplex::top_dimension_cofaces_tuples(m_mesh, v);

            // make sure that the current position is valid
            if (!inversion_invariant.after({}, tets)) {
                log_and_throw_error("Midpoint split im Marching component caused inversions.");
            }

            // sf_acc.scalar_attribute(v) = m_isovalue;
            // continue;

            // test position on the isovalue
            p = p_opt;
            if (u > 0 && u < 1 && inversion_invariant.after({}, tets)) {
                sf_acc.scalar_attribute(v) = m_isovalue;
                continue;
            }

            Eigen::VectorXd p_valid = p_mid;
            Eigen::VectorXd p_target = p_opt;

            // line search
            bool found_valid_position = false;
            for (size_t i = 0; i < m_linesearch_iterations; ++i) {
                p = 0.5 * (p_valid + p_target);


                if (inversion_invariant.after({}, tets)) {
                    p_valid = p;
                } else {
                    p_target = p;
                }
            }

            if (p_valid == p_mid) {
                logger().trace(
                    "Could not find a solution in the linesearch. Using the mid point: {}",
                    p_mid.transpose());
            }

            p = p_valid;
            {
                // compute isovalue
                const double l = (p1 - p0).norm();
                const double u_0 = (p - p0).norm() / l;
                const double u_1 = (p1 - p).norm() / l;
                const double sf_p = u_0 * sf0 + u_1 * sf1;
                sf_acc.scalar_attribute(v) = sf_p;
            }
        }
    }
}

void Marching::add_filter(const attribute::MeshAttributeHandle& label, const int64_t value)
{
    assert(label.primitive_type() == PrimitiveType::Edge);
    m_filter_labels.emplace_back(label);
    m_filter_values.emplace_back(value);
}

void Marching::add_pass_through(const attribute::MeshAttributeHandle& pass_through)
{
    m_pass_through_attributes.emplace_back(pass_through);
}

void Marching::add_pass_through(const std::vector<attribute::MeshAttributeHandle>& pass_through)
{
    m_pass_through_attributes.insert(
        m_pass_through_attributes.end(),
        pass_through.begin(),
        pass_through.end());
}

void Marching::add_isovalue(
    const attribute::MeshAttributeHandle& scalar_field,
    const double isovalue)
{
    m_scalar_field = scalar_field;
    m_isovalue = isovalue;
}

void Marching::set_isovalue_linesearch_iterations(const int64_t linesearch_iterations)
{
    m_linesearch_iterations = linesearch_iterations;
}

void Marching::invert_filter()
{
    m_invert_filter = true;
}


} // namespace wmtk::components
