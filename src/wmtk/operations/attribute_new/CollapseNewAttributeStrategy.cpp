#include "CollapseNewAttributeStrategy.hpp"
#include <wmtk/utils/primitive_range.hpp>


namespace wmtk::operations {

namespace {

template <typename T>
auto standard_collapse_strategy(CollapseBasicStrategy optype) -> CollapseFuncType<T>
{
    using VT = NewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case CollapseBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return standard_collapse_strategy<T>(CollapseBasicStrategy::Mean);
        } else {
            return standard_collapse_strategy<T>(CollapseBasicStrategy::CopyTuple);
        }
    case CollapseBasicStrategy::CopyTuple: return [](const VT& a, const VT&) -> VT { return a; };
    case CollapseBasicStrategy::CopyOther: return [](const VT&, const VT& b) -> VT { return b; };
    case CollapseBasicStrategy::Mean:
        return [](const VT& a, const VT& b) -> VT { return (a + b) / T(2); };
    case CollapseBasicStrategy::Throw:
        return [](const VT&, const VT&) -> VT {
            throw std::runtime_error("Collapse should have a new attribute");
        };
    case CollapseBasicStrategy::None: return {};
    case CollapseBasicStrategy::CopyFromPredicate:
        throw std::runtime_error("Invalid CopyFromPredicate");
    }
    return {};
}


} // namespace


template <typename T>
CollapseNewAttributeStrategy<T>::CollapseNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle<T>& h,
    Mesh& m)
    : NewAttributeStrategy()
    , m_handle(h)
    , m_collapse_op(nullptr)
{
    set_collapse_strategy(CollapseBasicStrategy::Throw);
}

void CollapseNewAttributeStrategy::update(
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
            auto merged_simps = m_topo_info.merged_simplices(return_data_variant, input_tuple, pt);
            auto new_simps = m_topo_info.new_simplices(return_data_variant, output_tuple, pt);


            assert(merged_simps.size() == new_simps.size());

            for (size_t s = 0; s < merged_simps.size(); ++s) {
                assign_collapsed(pt, merged_simps[s], new_simps[s]);
            }
        }
    }
}


template <typename T>
void CollapseNewAttributeStrategy<T>::assign_collapsed(
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

    const auto old_pred = this->evaluate_predicate(m_handle.mesh());

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);


    new_value = m_collapse_op(a, b, old_pred);
}


template <typename T>
void CollapseNewAttributeStrategy<T>::set_collapse_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}
template <typename T>
void CollapseNewAttributeStrategy<T>::set_collapse_strategy(CollapseBasicStrategy optype)
{
    set_collapse_strategy(standard_collapse_strategy<T>(t));
}


template <typename T>
Mesh& CollapseNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType CollapseNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void CollapseNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle<T>(m, m_handle);
}
template <typename T>
bool CollapseNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandleVariant& attr) const
{
    using HandleT = wmtk::attribute::MeshAttributeHandle<T>;

    if (!std::holds_alternative<HandleT>(attr)) return false;

    return std::get<HandleT>(attr) == m_handle;
}

template class CollapseNewAttributeStrategy<char>;
template class CollapseNewAttributeStrategy<int64_t>;
template class CollapseNewAttributeStrategy<double>;
template class CollapseNewAttributeStrategy<Rational>;

} // namespace wmtk::operations
