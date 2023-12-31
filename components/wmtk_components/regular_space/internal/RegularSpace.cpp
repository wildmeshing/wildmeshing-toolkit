#include "RegularSpace.hpp"

#include <wmtk/Scheduler.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>

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
        const MeshAttributeHandle<int64_t>& attribute,
        PrimitiveType ptype,
        int64_t val)
        : m_accessor(m.create_accessor(attribute))
        , m_ptype(ptype)
        , m_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;
};

void set_split_strategy(
    Mesh& m,
    attribute::AttributeInitializationHandle<int64_t>& attr,
    const operations::NewAttributeStrategy::SplitBasicStrategy& split,
    const operations::NewAttributeStrategy::SplitRibBasicStrategy& rib)
{
    // switch (m.top_simplex_type()) {
    // case PrimitiveType::Face: {
    //     attr.trimesh_standard_split_strategy().set_standard_split_strategy(split);
    //     attr.trimesh_standard_split_strategy().set_standard_split_rib_strategy(rib);
    //     break;
    // }
    // case PrimitiveType::Tetrahedron: {
    //     throw std::runtime_error("Implementation for tetrahedra meshes is incomplete");
    //    // attr.tetmesh_standard_split_strategy().set_standard_split_strategy(split);
    //    // attr.tetmesh_standard_split_strategy().set_standard_split_rib_strategy(rib);
    //    break;
    //}
    // default:
    //    throw std::runtime_error("Can only set split strategy for triangle and tetrahedra
    //    meshes.");
    //}
}

RegularSpace::RegularSpace(Mesh& mesh)
    : m_mesh(mesh)
{}

void RegularSpace::regularize_tags(
    const std::vector<std::tuple<std::string, int64_t, int64_t>>& tags)
{
    //// remove strategies and build new ones
    // m_mesh.m_split_strategies.clear();
    // m_mesh.m_collapse_strategies.clear();
    //
    //// TODO replace by get attribute
    // m_pos_attribute = std::make_unique<attribute::AttributeInitializationHandle<double>>(
    //     m_mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3, true));
    //
    // attribute::AttributeInitializationHandle<int64_t> todo_attribute =
    //     m_mesh.register_attribute<int64_t>("todo_edgesplit_same_handle",
    //     wmtk::PrimitiveType::Edge, 1);
    //
    //// set split position to mean
    //{
    //    m_pos_attribute->trimesh_standard_split_strategy().set_standard_split_rib_strategy(
    //        operations::NewAttributeStrategy::SplitRibBasicStrategy::Mean);
    //}
    //
    //// todo attribute is set to default value after split
    // set_split_strategy(
    //     m_mesh,
    //     todo_attribute,
    //     operations::NewAttributeStrategy::SplitBasicStrategy::None,
    //     operations::NewAttributeStrategy::SplitRibBasicStrategy::None);
    //
    // std::deque<TagAttribute> tag_attributes;
    // for (const auto& [name, ptype_id, tag_val] : tags) {
    //     attribute::AttributeInitializationHandle<int64_t> handle =
    //         m_mesh.register_attribute<int64_t>(name, get_primitive_type_from_id(ptype_id), 1,
    //         true);
    //
    //     TagAttribute& attr =
    //         tag_attributes.emplace_back(m_mesh, handle, handle.primitive_type(), tag_val);
    //
    //     const int64_t val = tag_val;
    //
    //     set_split_strategy(
    //         m_mesh,
    //         handle,
    //         operations::NewAttributeStrategy::SplitBasicStrategy::Copy,
    //         operations::NewAttributeStrategy::SplitRibBasicStrategy::None);
    // }
    //
    //// sort attributes
    //// std::sort(tag_attributes.begin(), tag_attributes.end(), [](TagAttribute& a, TagAttribute& b)
    //// {
    ////    return get_primitive_type_id(a.m_ptype) > get_primitive_type_id(b.m_ptype);
    ////});
    //
    //// make sure no primitive type exists more than once
    // assert(!tag_attributes.empty());
    //{
    //     for (size_t i = 1; i < tag_attributes.size(); ++i) {
    //         TagAttribute& a = tag_attributes[i - 1];
    //         TagAttribute& b = tag_attributes[i];
    //         assert(get_primitive_type_id(a.m_ptype) > get_primitive_type_id(b.m_ptype));
    //         if (get_primitive_type_id(a.m_ptype) <= get_primitive_type_id(b.m_ptype)) {
    //             log_and_throw_error("Tag array must be sorted in descending order");
    //         }
    //     }
    // }

    // TODO ...
    throw std::runtime_error("not fully implemented");
}

} // namespace wmtk::components::internal
