#include "SimplicialEmbedding.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TetCellSplit.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include <deque>

namespace wmtk::components::simplicial_embedding::internal {

namespace {

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
} // namespace

SimplicialEmbedding::SimplicialEmbedding(
    Mesh& mesh,
    const std::vector<attribute::MeshAttributeHandle>& label_attributes,
    const int64_t& value,
    const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes)
    : m_mesh(mesh)
    , m_label_attributes(label_attributes)
    , m_value(value)
    , m_pass_through_attributes(pass_through_attributes)
{}

void SimplicialEmbedding::regularize_tags(bool generate_simplicial_embedding)
{
    using namespace operations;

    std::vector<attribute::MeshAttributeHandle> todo_handles;
    for (size_t i = 0; i < m_label_attributes.size(); ++i) {
        attribute::MeshAttributeHandle todo_handle =
            m_mesh.register_attribute<int64_t>("todo", m_label_attributes[i].primitive_type(), 1);
        todo_handles.emplace_back(todo_handle);
    }

    std::deque<TagAttribute> tag_attributes;
    for (size_t i = 0; i < m_label_attributes.size(); ++i) {
        tag_attributes.emplace_back(
            m_mesh,
            m_label_attributes[i],
            m_label_attributes[i].primitive_type(),
            m_value);
    }

    // make sure tag vector is complete and sorted in descending order
    for (size_t i = 0; i < tag_attributes.size(); ++i) {
        TagAttribute& a = tag_attributes[i];
        if (get_primitive_type_id(a.m_ptype) != m_mesh.top_cell_dimension() - i) {
            log_and_throw_error(
                "Tag array must be sorted in descending order starting with "
                "the top simplex type up to vertex.");
        }
    }


    // tag all faces of attributes
    for (size_t attr_it = 0; attr_it < tag_attributes.size() - 1; ++attr_it) {
        const TagAttribute& ta = tag_attributes[attr_it];
        for (const Tuple& t : m_mesh.get_all(ta.m_ptype)) {
            if (ta.m_accessor.const_scalar_attribute(t) != ta.m_val) {
                continue; // t is not tagged
            }

            const PrimitiveType face_ptype =
                get_primitive_type_from_id(get_primitive_type_id(ta.m_ptype) - 1);
            const auto faces = simplex::faces_single_dimension_tuples(
                m_mesh,
                simplex::Simplex(m_mesh, ta.m_ptype, t),
                face_ptype);

            TagAttribute& face_ta = tag_attributes[attr_it + 1];
            for (const Tuple& f : faces) {
                face_ta.m_accessor.scalar_attribute(f) = face_ta.m_val;
            }
        }
    }

    if (!generate_simplicial_embedding) {
        return;
    }

    // check for inversions
    if (m_mesh.has_attribute<double>("vertices", PrimitiveType::Vertex)) {
        const auto pos_handle =
            m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        if (pos_handle.dimension() == m_mesh.top_cell_dimension() - 1) {
            SimplexInversionInvariant<double> inv(pos_handle.mesh(), pos_handle.as<double>());

            if (!inv.after({}, m_mesh.get_all(m_mesh.top_simplex_type()))) {
                logger().error("Mesh is not inversion free BEFORE simplicial embedding!");
            }
        }
    }

    // split untagged simplices that have only tagged faces
    for (size_t attr_it = 0; attr_it < tag_attributes.size() - 1;) {
        const TagAttribute& ta = tag_attributes[attr_it];

        attribute::MeshAttributeHandle& todo_handle = todo_handles[attr_it];
        auto todo_acc = m_mesh.create_accessor<int64_t>(todo_handle);

        int64_t count_todos = 0;
        for (const Tuple& t : m_mesh.get_all(ta.m_ptype)) {
            if (ta.m_accessor.const_scalar_attribute(t) == ta.m_val) {
                continue; // t is tagged
            }

            const PrimitiveType face_ptype =
                get_primitive_type_from_id(get_primitive_type_id(ta.m_ptype) - 1);
            const auto faces = simplex::faces_single_dimension_tuples(
                m_mesh,
                simplex::Simplex(m_mesh, ta.m_ptype, t),
                face_ptype);

            const TagAttribute& face_ta = tag_attributes[attr_it + 1];

            bool all_faces_are_tagged = true;

            for (const Tuple& f : faces) {
                if (face_ta.m_accessor.const_scalar_attribute(f) != face_ta.m_val) {
                    all_faces_are_tagged = false;
                    break;
                }
            }

            if (all_faces_are_tagged) {
                todo_acc.scalar_attribute(t) = 1;
                ++count_todos;
            }
        }

        // split simplex because all its faces are tagged
        Scheduler scheduler;
        int64_t count_done = 0;
        switch (ta.m_ptype) {
        case PrimitiveType::Edge: { // edge split
            EdgeSplit op_split(m_mesh);
            op_split.add_invariant(
                std::make_shared<TodoInvariant>(
                    m_mesh,
                    std::get<attribute::TypedAttributeHandle<int64_t>>(todo_handle.handle())));

            // todos
            for (const attribute::MeshAttributeHandle& h : todo_handles) {
                op_split.set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::None,
                    SplitRibBasicStrategy::None);
            }
            // labels
            for (const attribute::MeshAttributeHandle& h : m_label_attributes) {
                op_split.set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::Copy,
                    SplitRibBasicStrategy::None);
            }
            // pass_through
            for (const auto& attr : m_pass_through_attributes) {
                op_split.set_new_attribute_strategy(attr);
            }

            logger().info("split {} edges", count_todos);
            while (true) {
                const auto stats = scheduler.run_operation_on_all(op_split);
                count_done += stats.number_of_successful_operations();
                if (stats.number_of_successful_operations() == 0) {
                    break;
                }
            }

            break;
        }
        case PrimitiveType::Triangle: { // face split
            composite::TriFaceSplit op_face_split(m_mesh);
            op_face_split.add_invariant(
                std::make_shared<TodoInvariant>(
                    m_mesh,
                    std::get<attribute::TypedAttributeHandle<int64_t>>(todo_handle.handle())));

            // todos
            for (const attribute::MeshAttributeHandle& h : todo_handles) {
                op_face_split.split().set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::None,
                    SplitRibBasicStrategy::None);
                op_face_split.collapse().set_new_attribute_strategy(h, CollapseBasicStrategy::None);
            }
            // labels
            for (const attribute::MeshAttributeHandle& h : m_label_attributes) {
                op_face_split.split().set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::Copy,
                    SplitRibBasicStrategy::None);
                op_face_split.collapse().set_new_attribute_strategy(h, CollapseBasicStrategy::None);
            }
            // pass_through
            for (const auto& attr : m_pass_through_attributes) {
                op_face_split.split().set_new_attribute_strategy(attr);
                op_face_split.collapse().set_new_attribute_strategy(
                    attr,
                    CollapseBasicStrategy::None);
            }


            logger().info("split {} faces", count_todos);
            while (true) {
                const auto stats = scheduler.run_operation_on_all(op_face_split);
                count_done += stats.number_of_successful_operations();
                if (stats.number_of_successful_operations() == 0) {
                    break;
                }
            }

            break;
        }
        case PrimitiveType::Tetrahedron: { // tet split
            composite::TetCellSplit op_tet_split(m_mesh);
            op_tet_split.add_invariant(
                std::make_shared<TodoInvariant>(
                    m_mesh,
                    std::get<attribute::TypedAttributeHandle<int64_t>>(todo_handle.handle())));

            // todos
            for (const attribute::MeshAttributeHandle& h : todo_handles) {
                op_tet_split.split().set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::None,
                    SplitRibBasicStrategy::None);
                op_tet_split.collapse().set_new_attribute_strategy(h, CollapseBasicStrategy::None);
            }
            // labels
            for (const attribute::MeshAttributeHandle& h : m_label_attributes) {
                op_tet_split.split().set_new_attribute_strategy(
                    h,
                    SplitBasicStrategy::Copy,
                    SplitRibBasicStrategy::None);
                op_tet_split.collapse().set_new_attribute_strategy(h, CollapseBasicStrategy::None);
            }
            // pass_through
            for (const auto& attr : m_pass_through_attributes) {
                op_tet_split.split().set_new_attribute_strategy(attr);
                op_tet_split.collapse().set_new_attribute_strategy(
                    attr,
                    CollapseBasicStrategy::None);
            }

            logger().info("split {} tetrahedra", count_todos);
            while (true) {
                const auto stats = scheduler.run_operation_on_all(op_tet_split);
                count_done += stats.number_of_successful_operations();
                if (stats.number_of_successful_operations() == 0) {
                    break;
                }
            }

            break;
        }
        case PrimitiveType::Vertex: { // edge split
            log_and_throw_error("Cannot split point mesh");
            break;
        }
        default: log_and_throw_error("unknown primitive type"); break;
        }

        if (count_done == count_todos) {
            ++attr_it;
        } else {
            logger().info(
                "{} remain. Regularize same primitive type once more.",
                count_todos - count_done);
        }
    }

    // check for inversions
    if (m_mesh.has_attribute<double>("vertices", PrimitiveType::Vertex)) {
        const auto pos_handle =
            m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        SimplexInversionInvariant<double> inv(pos_handle.mesh(), pos_handle.as<double>());

        if (pos_handle.dimension() == m_mesh.top_cell_dimension() - 1) {
            SimplexInversionInvariant<double> inv(pos_handle.mesh(), pos_handle.as<double>());

            if (!inv.after({}, m_mesh.get_all(m_mesh.top_simplex_type()))) {
                logger().error("Mesh is not inversion free after simplicial embedding!");
            }
        }
    }
}


} // namespace wmtk::components::simplicial_embedding::internal
