#pragma once
#include <functional>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/attribute/MeshAttributeHandle.hpp>
namespace wmtk {
class Mesh;
class Rational;
class Tuple;
} // namespace wmtk

namespace wmtk::operations {


class NewAttributeStrategy
{
public:
    // default operation types, default specifies for rational/double we use averages , o/w copytuple
    enum class CollapseBasicStrategy {
        Default,
        CopyTuple,
        CopyOther, // per-dimension "other" simplex option
        CopyFromPredicate,
        Mean,
        None
    };
    // default operation types
    enum class SplitBasicStrategy { Default, Copy, Half, None };

    enum class BasicSimplexPredicate { Default, IsInterior };


    //rib and collapse have hte same prototypes / default funs available
    using SplitRibBasicStrategy = CollapseBasicStrategy;
    struct OpSettings
    {
        SplitRibBasicStrategy split_rib_optype = SplitRibBasicStrategy::Default;
        SplitBasicStrategy split_optype = SplitBasicStrategy::Default;
        CollapseBasicStrategy collapse_optype = CollapseBasicStrategy::Default;
    };

    template <typename T>
    using VecType = VectorX<T>;

    // given a k-simplex that was split in two, provide new values for thw two simplices created
    // from it first one is the one that "shares" a vertex with the op's "input tuple
    template <typename T, typename VT = VecType<T>>
    using SplitFuncType = std::function<std::array<VT, 2>(const VT&)>;

    // given two ear $k$-simplices, define a value for the single new $k$-simplex between them
    template <typename T, typename VT = VecType<T>>
    using SplitRibFuncType = std::function<VT(const VT&, const VT&)>;

    using SimplexPredicateType = std::function<bool(const simplex::Simplex&)>;

    // given two k-simplices that were merged into one, provide new values for that new simplex.
    // first argument is the one that "shares" a vertex with the op's "input tuple
    template <typename T, typename VT = VecType<T>>
    using CollapseFuncType = std::function<VT(const VT&, const VT&)>;


    template <typename T>
    static CollapseFuncType<T> standard_collapse_strategy(
        CollapseBasicStrategy = CollapseBasicStrategy::Default);
    template <typename T>
    static SplitFuncType<T> standard_split_strategy(
        SplitBasicStrategy = SplitBasicStrategy::Default);
    template <typename T>
    static SplitRibFuncType<T> standard_split_rib_strategy(
        SplitRibBasicStrategy = SplitRibBasicStrategy::Default);

    // can't speceify this without a way to capture the mesh :(
    // static SimplexPredicateType standard_prefix_predicate(
    //    BasicSimplexPredicate = BasicSimplexPredicate::Default);


    // same for split

    virtual ~NewAttributeStrategy();


    virtual bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const = 0;
    virtual void update_handle_mesh(Mesh&) = 0;

    virtual PrimitiveType primitive_type() const = 0;

    virtual Mesh& mesh() = 0;

    const Mesh& mesh() const;
};

} // namespace wmtk::operations
