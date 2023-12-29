#include "BasicSplitNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_standard_split_rib_strategy(SplitRibBasicStrategy t)
{
    set_split_rib_strategy(standard_split_rib_strategy<T>(t));
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_standard_split_strategy(SplitBasicStrategy t)
{
    set_split_strategy(standard_split_strategy<T>(t));
}
template <typename T>
BasicSplitNewAttributeStrategy<T>::BasicSplitNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h)
    : SplitNewAttributeStrategy(
          dynamic_cast<TriMesh&>(const_cast<wmtk::attribute::MeshAttributeHandle<T>&>(h).mesh()))
    , m_handle(h)
    , m_split_rib_op(nullptr)
    , m_split_op(nullptr)
{}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    if (!bool(m_split_rib_op)) {
        throw std::runtime_error("Rib split attribute needs to have a transfer");
    }
    if (pt != primitive_type()) {
        return;
    }

    auto acc = m_handle.create_accessor();
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_vector_attribute(input_ears[0]),
            acc.const_vector_attribute(input_ears[1]));
    });

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);
    new_value = m_split_rib_op(a, b);
}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::assign_split(
    PrimitiveType pt,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices)
{
    if (!bool(m_split_op)) {
        throw std::runtime_error("Spine split attribute needs to have a transfer");
    }
    if (pt != primitive_type()) {
        return;
    }
    auto acc = m_handle.create_accessor();
    const VecType old_value =
        m_handle.mesh().parent_scope([&]() { return acc.const_vector_attribute(input_simplex); });
    auto arr = m_split_op(old_value);
    for (size_t j = 0; j < 2; ++j) {
        acc.vector_attribute(split_simplices[j]) = arr[j];
    }
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_rib_strategy(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_strategy(SplitFuncType&& f)
{
    m_split_op = std::move(f);
}


template <typename T>
Mesh& BasicSplitNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType BasicSplitNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}

template <typename T>
bool BasicSplitNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}

template class BasicSplitNewAttributeStrategy<char>;
template class BasicSplitNewAttributeStrategy<long>;
template class BasicSplitNewAttributeStrategy<double>;
template class BasicSplitNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
