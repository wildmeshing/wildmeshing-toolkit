#include "SplitNewAttributeStrategy.hpp"
#include <wmtk/utils/primitive_range.hpp>


namespace wmtk::operations {

namespace {

template <typename T>
auto standard_split_strategy(SplitBasicStrategy optype) -> SplitFuncType<T>
{
    using VT = SplitNewAttributeStrategy::VecType<T>;

    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        return [](const VT& a) -> std::array<VT, 2> { return std::array<VT, 2>{{a, a}}; };
    case SplitBasicStrategy::Half:
        return [](const VT& a) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a / T(2), a / T(2)}};
        };
    case SplitBasicStrategy::Throw:
        return [](const VT&) -> std::array<VT, 2> {
            throw std::runtime_error("Split should have a new attribute");
        };
    case SplitBasicStrategy::None: return {};
    }
    return {};
}
template <typename T>
auto standard_split_rib_strategy(SplitRibBasicStrategy optype) -> SplitRibFuncType<T>
{
    using VT = SplitNewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case SplitRibBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return standard_split_rib_strategy<T>(SplitRibBasicStrategy::Mean);
        } else {
            return standard_split_rib_strategy<T>(SplitRibBasicStrategy::CopyTuple);
        }
    case SplitRibBasicStrategy::CopyTuple: return [](const VT& a, const VT&) -> VT { return a; };
    case SplitRibBasicStrategy::CopyOther: return [](const VT&, const VT& b) -> VT { return b; };
    case SplitRibBasicStrategy::Mean:
        return [](const VT& a, const VT& b) -> VT { return (a + b) / T(2); };
    case SplitRibBasicStrategy::Throw:
        return [](const VT&, const VT&) -> VT {
            throw std::runtime_error("Split should have a new attribute");
        };
    case SplitRibBasicStrategy::None: return {};
    case CollapseBasicStrategy::CopyFromPredicate:
        throw std::runtime_error("Invalid CopyFromPredicate");
    }
    return {};
}
} // namespace


template <typename T>
SplitNewAttributeStrategy<T>::SplitNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h)
    : NewAttributeStrategy()
    , m_handle(h)
    , m_split_rib_op(nullptr)
    , m_split_op(nullptr)
{
    set_split_rib_strategy(SplitRibBasicStrategy::Throw);
    set_split_strategy(SplitBasicStrategy::Throw);
}

template <typename T>
void SplitNewAttributeStrategy<T>::update(
    const ReturnData& data,
    const OperationTupleData& op_datas)
{
    assert(op_datas.find(&mesh()) != op_datas.end());
    const std::vector<std::array<Tuple, 2>>& tuple_pairs = op_datas.at(&mesh());

    for (const auto& tuple_pair : tuple_pairs) {
        const Tuple& input_tuple = tuple_pair[0];
        const Tuple& output_tuple = tuple_pair[1];

        const auto& return_data_variant =
            data.get_variant(mesh(), wmtk::simplex::Simplex::edge(input_tuple));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type())) {
            {
                auto old_simps =
                    m_topo_info.input_ear_simplices(return_data_variant, input_tuple, pt);
                auto new_simps =
                    m_topo_info.output_rib_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split_ribs(pt, old_simps[s], new_simps[s]);
                }
            }
            {
                auto old_simps =
                    m_topo_info.input_split_simplices(return_data_variant, input_tuple, pt);
                auto new_simps =
                    m_topo_info.output_split_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split(pt, old_simps[s], new_simps[s]);
                }
            }
        }
    }
}


template <typename T>
void SplitNewAttributeStrategy<T>::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    if (!bool(m_split_rib_op)) {
        return;
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

    const auto old_pred = this->evaluate_predicate(m_handle.mesh());

    new_value = m_split_rib_op(a, b, old_pred);
}

template <typename T>
void SplitNewAttributeStrategy<T>::assign_split(
    PrimitiveType pt,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices)
{
    if (!bool(m_split_rib_op)) {
        return;
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

    const auto old_pred = this->evaluate_predicate(m_handle.mesh());

    new_value = m_split_rib_op(a, b, old_pred);
}


template <typename T>
void SplitNewAttributeStrategy<T>::set_split_rib_strategy(SplitRibBasicStrategy t)
{
    set_split_rib_strategy(standard_split_rib_strategy<T>(t));
}
template <typename T>
void SplitNewAttributeStrategy<T>::set_split_strategy(SplitBasicStrategy t)
{
    set_split_strategy(standard_split_strategy<T>(t));
}

template <typename T>
void SplitNewAttributeStrategy<T>::set_split_rib_strategy(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
}
template <typename T>
void SplitNewAttributeStrategy<T>::set_split_strategy(SplitFuncType&& f)
{
    m_split_op = std::move(f);
}

template <typename T>
Mesh& SplitNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}

template <typename T>
PrimitiveType SplitNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}

template <typename T>
void SplitNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}

template <typename T>
bool SplitNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}


template class SplitNewAttributeStrategy<char>;
template class SplitNewAttributeStrategy<int64_t>;
template class SplitNewAttributeStrategy<double>;
template class SplitNewAttributeStrategy<Rational>;

} // namespace wmtk::operations
