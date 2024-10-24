
#include "AttributeTransferStrategyBase.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/neighbors_single_dimension.hpp>

#include <wmtk/simplex/utils/unique_homogeneous_simplices.hpp>

namespace wmtk::operations {
AttributeTransferStrategyBase::AttributeTransferStrategyBase(
    const attribute::MeshAttributeHandle& my_handle)
    : m_handle(my_handle)
{}
AttributeTransferStrategyBase::~AttributeTransferStrategyBase() = default;
const Mesh& AttributeTransferStrategyBase::mesh() const
{
    return const_cast<const Mesh&>(const_cast<AttributeTransferStrategyBase*>(this)->mesh());
}

void AttributeTransferStrategyBase::run_on_all() const
{
    const PrimitiveType pt = m_handle.primitive_type();
    auto tuples = m_handle.mesh().get_all(pt);

    for (const Tuple& t : tuples) {
        run(simplex::Simplex(m_handle.mesh(), pt, t));
    }
}

std::vector<Tuple> AttributeTransferStrategyBase::get_parent_simplices(
    const attribute::MeshAttributeHandle& me,
    const attribute::MeshAttributeHandle& parent,
    const simplex::Simplex& s)
{
    return get_parent_simplices(me.mesh(), parent.mesh(), s, parent.primitive_type());
}

std::vector<Tuple> AttributeTransferStrategyBase::get_parent_simplices(
    const Mesh& m,
    const Mesh& parent,
    const simplex::Simplex& s,
    PrimitiveType parent_primitive_type)
{
    const PrimitiveType my_primitive_type = s.primitive_type();


    // the set of simplices that can be traversed without crossing a mesh-mesh edge more than once
    std::vector<Tuple> parent_tuples = m.lub_map_tuples(parent, s);

    if (my_primitive_type != parent_primitive_type) {
        // lambda for running either of the cases
        std::vector<Tuple> r;
        if (parent_tuples.size() == 1) {
            r = simplex::neighbors_single_dimension_tuples(
                m,
                simplex::Simplex(m, my_primitive_type, parent_tuples[0]),
                parent_primitive_type);
        } else {
            for (const auto& parent_tup : parent_tuples) {
                std::vector<Tuple> c = simplex::neighbors_single_dimension_tuples(
                    m,
                    simplex::Simplex(m, my_primitive_type, parent_tup),
                    parent_primitive_type);
                std::copy(c.begin(), c.end(), std::back_inserter(r));
            }
            if (parent_tuples.size() > 1) {
                simplex::utils::unique_homogeneous_simplices_inline(m, parent_primitive_type, r);
            }
        }
        parent_tuples = std::move(r);
    }
    return parent_tuples;
}

bool AttributeTransferStrategyBase::matches_attribute(
    const wmtk::attribute::MeshAttributeHandle& attr) const
{
    return handle() == attr;
}
} // namespace wmtk::operations
