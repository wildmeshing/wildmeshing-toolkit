#pragma once
namespace wmtk::operations {
// default operation types
enum class SplitBasicStrategy { Default, Copy, Half, Throw, None };
//rib and collapse have hte same prototypes / default funs available
enum class SplitRibBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Min,
    Throw,
    None
};
// default operation types, default specifies for rational/double we use averages , o/w copytuple
enum class CollapseBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Throw,
    None
};
} // namespace wmtk::operations
