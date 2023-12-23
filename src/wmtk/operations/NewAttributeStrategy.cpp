#include "NewAttributeStrategy.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::operations {

NewAttributeStrategy::~NewAttributeStrategy() = default;


const Mesh& NewAttributeStrategy::mesh() const
{
    return const_cast<const Mesh&>(const_cast<NewAttributeStrategy*>(this)->mesh());
}

template <typename T>
auto NewAttributeStrategy::standard_collapse_strategy(CollapseBasicStrategy optype)
    -> CollapseFuncType<T>
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
    case CollapseBasicStrategy::None: return {};
    }
    return {};
}
template <typename T>
auto NewAttributeStrategy::standard_split_strategy(SplitBasicStrategy optype) -> SplitFuncType<T>
{
    using VT = NewAttributeStrategy::VecType<T>;
    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        return [](const VT& a) -> std::array<VT, 2> { return std::array<VT, 2>{{a, a}}; };
    case SplitBasicStrategy::Half:
        return [](const VT& a) -> std::array<VT, 2> { return std::array<VT, 2>{{a/T(2), a/T(2)}}; };
    case SplitBasicStrategy::None: return {};
    }
    return {};
}
template <typename T>
auto NewAttributeStrategy::standard_split_rib_strategy(SplitRibBasicStrategy optype)
    -> SplitRibFuncType<T>
{
    using VT = NewAttributeStrategy::VecType<T>;
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
    case SplitRibBasicStrategy::None: return {};
    }
    return {};
}

template auto NewAttributeStrategy::standard_collapse_strategy<double>(CollapseBasicStrategy optype)
    -> CollapseFuncType<double>;
template auto NewAttributeStrategy::standard_split_strategy<double>(SplitBasicStrategy optype)
    -> SplitFuncType<double>;
template auto NewAttributeStrategy::standard_split_rib_strategy<double>(
    SplitRibBasicStrategy optype) -> SplitRibFuncType<double>;
template auto NewAttributeStrategy::standard_collapse_strategy<long>(CollapseBasicStrategy optype)
    -> CollapseFuncType<long>;
template auto NewAttributeStrategy::standard_split_strategy<long>(SplitBasicStrategy optype)
    -> SplitFuncType<long>;
template auto NewAttributeStrategy::standard_split_rib_strategy<long>(SplitRibBasicStrategy optype)
    -> SplitRibFuncType<long>;
template auto NewAttributeStrategy::standard_collapse_strategy<char>(CollapseBasicStrategy optype)
    -> CollapseFuncType<char>;
template auto NewAttributeStrategy::standard_split_strategy<char>(SplitBasicStrategy optype)
    -> SplitFuncType<char>;
template auto NewAttributeStrategy::standard_split_rib_strategy<char>(SplitRibBasicStrategy optype)
    -> SplitRibFuncType<char>;
template auto NewAttributeStrategy::standard_collapse_strategy<Rational>(
    CollapseBasicStrategy optype) -> CollapseFuncType<Rational>;
template auto NewAttributeStrategy::standard_split_strategy<Rational>(SplitBasicStrategy optype)
    -> SplitFuncType<Rational>;
template auto NewAttributeStrategy::standard_split_rib_strategy<Rational>(
    SplitRibBasicStrategy optype) -> SplitRibFuncType<Rational>;
} // namespace wmtk::operations
