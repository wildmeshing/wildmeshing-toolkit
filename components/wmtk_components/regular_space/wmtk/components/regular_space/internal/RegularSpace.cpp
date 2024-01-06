#include "RegularSpace.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include <deque>

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
        : m_accessor(m.create_accessor<int64_t>(attribute))
        , m_ptype(ptype)
        , m_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;
};

RegularSpace::RegularSpace(Mesh& mesh)
    : m_mesh(mesh)
{}

void RegularSpace::regularize_tags(
    const std::vector<std::tuple<std::string, int64_t, int64_t>>& tags)
{
    using namespace operations;

    m_pos_attribute = m_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    std::vector<attribute::TypedAttributeHandle<int64_t>> tag_handles;
    std::vector<attribute::TypedAttributeHandle<int64_t>> todo_handles;
    std::deque<TagAttribute> tag_attributes;
    for (const auto& [name, ptype_id, tag_val] : tags) {
        attribute::TypedAttributeHandle<int64_t> tag_handle =
            m_mesh.get_attribute_handle<int64_t>(name, get_primitive_type_from_id(ptype_id))
                .as<int64_t>();

        tag_handles.emplace_back(tag_handle);

        attribute::TypedAttributeHandle<int64_t> todo_handle =
            m_mesh.register_attribute<int64_t>("todo", tag_handle.primitive_type(), 1)
                .as<int64_t>();
        todo_handles.emplace_back(todo_handle);

        TagAttribute& attr =
            tag_attributes.emplace_back(m_mesh, tag_handle, tag_handle.primitive_type(), tag_val);

        const int64_t val = tag_val;
    }

    // make sure tag vector is complete and sorted in descending order
    for (size_t i = 0; i < tag_attributes.size(); ++i) {
        TagAttribute& a = tag_attributes[i];
        if (get_primitive_type_id(a.m_ptype) != m_mesh.top_cell_dimension() - i) {
            log_and_throw_error("Tag array must be sorted in descending order starting with "
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
                simplex::Simplex(ta.m_ptype, t),
                face_ptype);

            TagAttribute& face_ta = tag_attributes[attr_it + 1];
            for (const Tuple& f : faces) {
                face_ta.m_accessor.scalar_attribute(f) = face_ta.m_val;
            }
        }
    }

    // split untagged simplices that have only tagged faces
    for (size_t attr_it = 0; attr_it < tag_attributes.size() - 1; ++attr_it) {
        const TagAttribute& ta = tag_attributes[attr_it];

        attribute::TypedAttributeHandle<int64_t>& todo_handle = todo_handles[attr_it];
        auto todo_acc = m_mesh.create_accessor(todo_handle);

        for (const Tuple& t : m_mesh.get_all(ta.m_ptype)) {
            if (ta.m_accessor.const_scalar_attribute(t) == ta.m_val) {
                continue; // t is tagged
            }

            const PrimitiveType face_ptype =
                get_primitive_type_from_id(get_primitive_type_id(ta.m_ptype) - 1);
            const auto faces = simplex::faces_single_dimension_tuples(
                m_mesh,
                simplex::Simplex(ta.m_ptype, t),
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
            }
        }

        if (m_mesh.top_simplex_type() == PrimitiveType::Tetrahedron) {
            log_and_throw_error("Regular space component not implemented for TetMeshes.");
        }

        // split simplex because all its faces are tagged
        Scheduler scheduler;
        switch (ta.m_ptype) {
        case PrimitiveType::Edge: { // edge split
            EdgeSplit op_split(m_mesh);
            op_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_handle));

            for (const attribute::TypedAttributeHandle<int64_t>& h : todo_handles) {
                op_split.set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    SplitBasicStrategy::None,
                    SplitRibBasicStrategy::None);
            }

            op_split.set_new_attribute_strategy(
                *m_pos_attribute,
                SplitBasicStrategy::None,
                SplitRibBasicStrategy::Mean);

            for (const attribute::TypedAttributeHandle<int64_t>& h : tag_handles) {
                op_split.set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    SplitBasicStrategy::Copy,
                    SplitRibBasicStrategy::None);
            }

            while (true) {
                const auto stats = scheduler.run_operation_on_all(op_split);
                if (stats.number_of_successful_operations() == 0) {
                    break;
                }
            }

            break;
        }
        case PrimitiveType::Face: { // face split
            composite::TriFaceSplit op_face_split(m_mesh);
            op_face_split.add_invariant(std::make_shared<TodoInvariant>(m_mesh, todo_handle));

            for (const attribute::TypedAttributeHandle<int64_t>& h : todo_handles) {
                op_face_split.split().set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    SplitBasicStrategy::None,
                    SplitRibBasicStrategy::None);
                op_face_split.collapse().set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    CollapseBasicStrategy::None);
            }


            op_face_split.split().set_new_attribute_strategy(
                *m_pos_attribute,
                SplitBasicStrategy::None,
                SplitRibBasicStrategy::Mean);
            op_face_split.collapse().set_new_attribute_strategy(
                *m_pos_attribute,
                CollapseBasicStrategy::CopyOther);


            for (const attribute::TypedAttributeHandle<int64_t>& h : tag_handles) {
                op_face_split.split().set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    SplitBasicStrategy::Copy,
                    SplitRibBasicStrategy::None);
                op_face_split.collapse().set_new_attribute_strategy(
                    attribute::MeshAttributeHandle(m_mesh, h),
                    CollapseBasicStrategy::None);
            }

            while (true) {
                const auto stats = scheduler.run_operation_on_all(op_face_split);
                if (stats.number_of_successful_operations() == 0) {
                    break;
                }
            }

            break;
        }
        case PrimitiveType::Tetrahedron: {
            log_and_throw_error("Regular space component not implemented for TetMeshes.");
        }
        default: log_and_throw_error("unknown primitive type: {}", ta.m_ptype); break;
        }
    }


    // clear attributes
    std::vector<attribute::TypedAttributeHandleVariant> keep_attributes;
    keep_attributes.emplace_back(m_pos_attribute->handle());
    for (const attribute::TypedAttributeHandle<int64_t>& h : tag_handles) {
        keep_attributes.emplace_back(h);
    }
    m_mesh.clear_attributes(keep_attributes);
}

} // namespace wmtk::components::internal
