
#include "BasicCollapseNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
BasicCollapseNewAttributeStrategy<T>::BasicCollapseNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : CollapseNewAttributeStrategy(dynamic_cast<TriMesh&>(h.mesh()))
    , m_handle(h)
{}

template <typename T>
void BasicCollapseNewAttributeStrategy<T>::assign_collapsed(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_simplices,
    const Tuple& final_simplex)
{
    auto acc = m_handle.create_accessor();
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.vector_attribute(input_simplices[0]),
            acc.vector_attribute(input_simplices[1]));
    });

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);
    switch (m_optype) {
    case OpType::CopyOther: new_value = b; break;
    case OpType::Mean: new_value = (a + b) / 2; break;
    case OpType::Custom: new_value = m_collapse_op(a, b); break;
    default:
    case OpType::CopyTuple: new_value = a; break;
    }
}

template <typename T>
void BasicCollapseNewAttributeStrategy<T>::set_collapse_func(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}

template <typename T>
void BasicCollapseNewAttributeStrategy<T>::set_collapse_type(OpType t)
{
    m_optype = t;
}


template class BasicCollapseNewAttributeStrategy<char>;
template class BasicCollapseNewAttributeStrategy<long>;
template class BasicCollapseNewAttributeStrategy<double>;
template class BasicCollapseNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
