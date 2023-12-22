

#include <wmtk/utils/Rational.hpp>
#include "BoundaryAwayCollapseNewAttributeStrategy.hpp"


namespace wmtk::operations::tri_mesh {
template <typename T>
void BoundaryAwayCollapseNewAttributeStrategy<T>::set_standard_collapse_strategy(
    CollapseBoundaryAwayStrategy t)
{
    set_collapse_strategy(standard_collapse_strategy<T>(t));
}

template <typename T>
BoundaryAwayCollapseNewAttributeStrategy<T>::BoundaryAwayCollapseNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : CollapseNewAttributeStrategy(dynamic_cast<TriMesh&>(h.mesh()))
    , m_handle(h)
    , m_collapse_op(standard_collapse_strategy<T>())
{}

template <typename T>
Mesh& BoundaryAwayCollapseNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType BoundaryAwayCollapseNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void BoundaryAwayCollapseNewAttributeStrategy<T>::assign_collapsed(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_simplices,
    const Tuple& final_simplex)
{
    if (pt != primitive_type()) {
        return;
    }
    auto acc = m_handle.create_accessor();
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_vector_attribute(input_simplices[0]),
            acc.const_vector_attribute(input_simplices[1]));
    });


    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);


    new_value = m_collapse_op(a, b);
}

template <typename T>
void BoundaryAwayCollapseNewAttributeStrategy<T>::set_collapse_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}


template <typename T>
void BoundaryAwayCollapseNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}

template class BoundaryAwayCollapseNewAttributeStrategy<char>;
template class BoundaryAwayCollapseNewAttributeStrategy<long>;
template class BoundaryAwayCollapseNewAttributeStrategy<double>;
template class BoundaryAwayCollapseNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
