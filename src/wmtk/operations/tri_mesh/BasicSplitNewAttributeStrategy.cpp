#include "BasicSplitNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
BasicSplitNewAttributeStrategy<T>::BasicSplitNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : SplitNewAttributeStrategy(dynamic_cast<TriMesh&>(h.mesh()))
    , m_handle(h)
{}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    auto acc = m_handle.create_accessor();
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.vector_attribute(input_ears[0]),
            acc.vector_attribute(input_ears[1]));
    });

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);
    switch (m_split_ribs_optype) {
    case OpType::CopyOther: new_value = b; break;
    case OpType::Mean: new_value = (a + b) / 2; break;
    case OpType::Custom: new_value = m_split_rib_op(a, b); break;
    default:
    case OpType::CopyTuple: new_value = a; break;
    }
}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::assign_split(
    PrimitiveType pt,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices)
{
    auto acc = m_handle.create_accessor();
    const VecType old_value =
        m_handle.mesh().parent_scope([&]() { return acc.vector_attribute(input_simplex); });
    switch (m_split_optype) {
    case OpType::Custom: {
        auto arr = m_split_op(old_value);
        for (size_t j = 0; j < 2; ++j) {
            acc.vector_attribute(split_simplices[j]) = arr[j];
        }
        break;
    }
    case OpType::Mean: [[fallthrough]];
    case OpType::CopyTuple: [[fallthrough]];
    case OpType::CopyOther: [[fallthrough]];
    default:
        for (const Tuple& t : split_simplices) {
            acc.vector_attribute(t) = old_value;
        }
    }
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_rib_func(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_func(SplitFuncType&& f)
{
    m_split_op = std::move(f);
}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_rib_type(OpType t)
{
    m_split_ribs_optype = t;
}
template <typename T>
void BasicSplitNewAttributeStrategy<T>::set_split_type(OpType t)
{
    m_split_optype = t;
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

template class BasicSplitNewAttributeStrategy<char>;
template class BasicSplitNewAttributeStrategy<long>;
template class BasicSplitNewAttributeStrategy<double>;
template class BasicSplitNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
