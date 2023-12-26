#include "PredicateAwareCollapseNewAttributeStrategy.hpp"
#include <wmtk/utils/Rational.hpp>


namespace wmtk::operations::tri_mesh {

template <typename T>
PredicateAwareCollapseNewAttributeStrategy<T>::PredicateAwareCollapseNewAttributeStrategy(
    wmtk::attribute::MeshAttributeHandle<T>& h)
    : PredicateAwareCollapseNewAttributeStrategy(h, h.mesh())
{}

template <typename T>
PredicateAwareCollapseNewAttributeStrategy<T>::PredicateAwareCollapseNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h,
    Mesh& m)
    : CollapseNewAttributeStrategy(dynamic_cast<TriMesh&>(m))
    , m_handle(h)
//, m_collapse_op(standard_collapse_strategy<T>())
{}

template <typename T>
Mesh& PredicateAwareCollapseNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType PredicateAwareCollapseNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::assign_collapsed(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_simplices,
    const Tuple& final_simplex)
{
    if (!bool(m_collapse_op)) {
        return;
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
    auto old_pred = m_handle.mesh().parent_scope([&]() {
        std::bitset<2> pred(0);
        if (bool(m_simplex_predicate)) {
            for (size_t j = 0; j < 2; ++j) {
                pred[j] = m_simplex_predicate(simplex::Simplex(pt, input_simplices[j]));
            }
        }
        return pred;
    });


    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);


    new_value = m_collapse_op(a, b, old_pred);
}

template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::set_collapse_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}

template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::set_simplex_predicate(SimplexPredicateType&& f)
{
    m_simplex_predicate = std::move(f);
}

template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}
template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::set_standard_collapse_strategy(
    CollapseBasicStrategy optype)
{
    using VT = NewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case CollapseBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return set_standard_collapse_strategy(CollapseBasicStrategy::Mean);
        } else {
            return set_standard_collapse_strategy(CollapseBasicStrategy::CopyTuple);
        }
    case CollapseBasicStrategy::CopyTuple:
        set_collapse_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            // if both are boundary then return a (failed link anyway but oh well)
            // if a is boundary but b is interior get b though
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        });
        break;
    case CollapseBasicStrategy::CopyOther:
        set_collapse_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        });
        break;
    case CollapseBasicStrategy::Mean:
        set_collapse_strategy([&](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (bs[0] == bs[1]) {
                return (a + b) / T(2);
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        });
        break;
    case CollapseBasicStrategy::None: set_collapse_strategy(nullptr); break;
    }
}
template <typename T>
void PredicateAwareCollapseNewAttributeStrategy<T>::set_standard_simplex_predicate(
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
bool PredicateAwareCollapseNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}

template class PredicateAwareCollapseNewAttributeStrategy<char>;
template class PredicateAwareCollapseNewAttributeStrategy<long>;
template class PredicateAwareCollapseNewAttributeStrategy<double>;
template class PredicateAwareCollapseNewAttributeStrategy<Rational>;
} // namespace wmtk::operations::tri_mesh
