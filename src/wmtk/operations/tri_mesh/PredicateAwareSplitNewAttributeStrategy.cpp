#include "PredicateAwareSplitNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/TupleInspector.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
PredicateAwareSplitNewAttributeStrategy<T>::PredicateAwareSplitNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : PredicateAwareSplitNewAttributeStrategy(h, h.mesh())
{}

template <typename T>
PredicateAwareSplitNewAttributeStrategy<T>::PredicateAwareSplitNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h,
    Mesh& m)
    : SplitNewAttributeStrategy(dynamic_cast<TriMesh&>(m))
    , m_handle(h)
    , m_split_rib_op(nullptr)
    , m_split_op(nullptr)
{}

template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    if (!bool(m_split_rib_op)) {
        throw std::runtime_error("Attribute needs to have a transfer");
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

    auto old_pred = m_handle.mesh().parent_scope([&]() {
        std::bitset<2> pred(0);
        if (bool(m_simplex_predicate)) {
            for (size_t j = 0; j < 2; ++j) {
                pred[j] = m_simplex_predicate(simplex::Simplex(pt, input_ears[j]));
            }
        }
        return pred;
    });

    new_value = m_split_rib_op(a, b, old_pred);
}

template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::assign_split(
    PrimitiveType pt,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices)
{
    if (!bool(m_split_op)) {
        throw std::runtime_error("Attribute needs to have a transfer");
    }
    if (pt != primitive_type()) {
        return;
    }
    auto acc = m_handle.create_accessor();
    const VecType old_value =
        m_handle.mesh().parent_scope([&]() { return acc.const_vector_attribute(input_simplex); });

    std::bitset<2> pred(0);
    if (bool(m_simplex_predicate)) {
        for (size_t j = 0; j < 2; ++j) {
            pred[j] = m_simplex_predicate(simplex::Simplex(pt, split_simplices[j]));
        }
    }

    auto arr = m_split_op(old_value, pred);
    for (size_t j = 0; j < 2; ++j) {
        acc.vector_attribute(split_simplices[j]) = arr[j];
    }
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_split_rib_strategy(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_split_strategy(SplitFuncType&& f)
{
    m_split_op = std::move(f);
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_simplex_predicate(SimplexPredicateType&& f)
{
    m_simplex_predicate = std::move(f);
}

template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_standard_split_rib_strategy(
    SplitRibBasicStrategy optype)
{
    using VT = NewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case SplitRibBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return set_standard_split_rib_strategy(SplitRibBasicStrategy::Mean);
        } else {
            return set_standard_split_rib_strategy(SplitRibBasicStrategy::CopyTuple);
        }
    case SplitRibBasicStrategy::CopyTuple:
        set_split_rib_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            // if both are boundary then return a (failed link anyway but oh well)
            // if a is boundary but b is interior get b though
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        });
        break;
    case SplitRibBasicStrategy::CopyOther:
        set_split_rib_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        });
        break;
    case SplitRibBasicStrategy::Mean:
        set_split_rib_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (bs[0] == bs[1]) {
                return (a + b) / T(2);
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        });
        break;
    case SplitRibBasicStrategy::None: set_split_rib_strategy(nullptr); break;
    }
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_standard_split_strategy(
    SplitBasicStrategy optype)
{
    // Should this even exist? not clear
    using VT = NewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        set_split_strategy([](const VT& a, const std::bitset<2>& bs) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a, a}};
        });
        break;
    case SplitBasicStrategy::Half:
        set_split_strategy([](const VT& a, const std::bitset<2>& bs) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a / T(2), a / T(2)}};
        });
        break;
    case SplitBasicStrategy::None: set_split_strategy(nullptr);
    }
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::set_standard_simplex_predicate(
    BasicSimplexPredicate optype)
{
    switch (optype) {
    default: [[fallthrough]];
    case BasicSimplexPredicate::Default: [[fallthrough]];
    case BasicSimplexPredicate::IsInterior:
        set_simplex_predicate(
            [&](const simplex::Simplex& s) -> bool { return !mesh().is_boundary(s); });
        break;
    }
}

template <typename T>
Mesh& PredicateAwareSplitNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType PredicateAwareSplitNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void PredicateAwareSplitNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}

template <typename T>
bool PredicateAwareSplitNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}

template class PredicateAwareSplitNewAttributeStrategy<char>;
template class PredicateAwareSplitNewAttributeStrategy<long>;
template class PredicateAwareSplitNewAttributeStrategy<double>;
template class PredicateAwareSplitNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
