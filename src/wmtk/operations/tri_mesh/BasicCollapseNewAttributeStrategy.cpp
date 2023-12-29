
#include "BasicCollapseNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::operations::tri_mesh {
template <typename T>
void BasicCollapseNewAttributeStrategy<T>::set_standard_collapse_strategy(CollapseBasicStrategy t)
{
    set_collapse_strategy(standard_collapse_strategy<T>(t));
}

template <typename T>
BasicCollapseNewAttributeStrategy<T>::BasicCollapseNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h)
    : CollapseNewAttributeStrategy(
          dynamic_cast<TriMesh&>(const_cast<wmtk::attribute::MeshAttributeHandle<T>&>(h).mesh()))
    , m_handle(h)
    , m_collapse_op(nullptr)
{}

template <typename T>
Mesh& BasicCollapseNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType BasicCollapseNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void BasicCollapseNewAttributeStrategy<T>::assign_collapsed(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_simplices,
    const Tuple& final_simplex)
{
    if (!bool(m_collapse_op)) {
        throw std::runtime_error("Collapse attribute needs to have a transfer");
    }
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
void BasicCollapseNewAttributeStrategy<T>::set_collapse_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}


template <typename T>
void BasicCollapseNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}

template <typename T>
bool BasicCollapseNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}

template class BasicCollapseNewAttributeStrategy<char>;
template class BasicCollapseNewAttributeStrategy<long>;
template class BasicCollapseNewAttributeStrategy<double>;
template class BasicCollapseNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
