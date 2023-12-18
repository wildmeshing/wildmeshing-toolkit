#include "BasicSplitNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
BasicSplitNewAttributeStrategy<T>::BasicSplitNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : SplitNewAttributeStrategy(dynamic_cast<TriMesh&>(h.mesh()))
    , m_handle(h)
{
    m_split_optype = OpType::CopyTuple; // only copytuple makes sense

    m_split_ribs_optype = std::is_same_v<T, double> ? OpType::Mean : OpType::CopyTuple;
}

template <typename T>
void BasicSplitNewAttributeStrategy<T>::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    auto old_ids = mesh().parent_scope([&]() {
        std::array<long, 2> r;
        for (size_t j = 0; j < 2; ++j) {
            r[j] = mesh().id(input_ears[j], pt);
        }
        return r;
    });
    spdlog::info(
        "{} autoupdate: {} + {} => {} +++++ {} + {} => {}",
        mesh().get_attribute_name(m_handle),
        wmtk::utils::TupleInspector::as_string(input_ears[0]),
        wmtk::utils::TupleInspector::as_string(input_ears[1]),
        wmtk::utils::TupleInspector::as_string(final_simplex),
        old_ids[0],
        old_ids[1],
        mesh().id(final_simplex, pt));
    return;
    auto acc = m_handle.create_accessor();
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_vector_attribute(input_ears[0]),
            acc.const_vector_attribute(input_ears[1]));
    });

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);
    if constexpr (!std::is_same_v<T, wmtk::Rational>) {
        std::cout << a.transpose() << ":" << b.transpose() << "=>" << new_value.transpose()
                  << std::endl;
    }
    switch (m_split_ribs_optype) {
    case OpType::Default: break;
    case OpType::CopyOther:
        spdlog::info("Copyother");
        new_value = b;
        break;
    case OpType::Mean:
        spdlog::info("mean");
        new_value = (a + b) / 2;
        break;
    case OpType::Custom:
        spdlog::info("custom");
        new_value = m_split_rib_op(a, b);
        break;
    default:
    case OpType::CopyTuple:
        spdlog::info("copytuple");
        new_value = a;
        break;
    }
    if constexpr (!std::is_same_v<T, wmtk::Rational>) {
        std::cout << "==>" << new_value.transpose() << std::endl;
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
        m_handle.mesh().parent_scope([&]() { return acc.const_vector_attribute(input_simplex); });
    switch (m_split_optype) {
    case OpType::Default: break;
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

template class BasicSplitNewAttributeStrategy<char>;
template class BasicSplitNewAttributeStrategy<long>;
template class BasicSplitNewAttributeStrategy<double>;
template class BasicSplitNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
