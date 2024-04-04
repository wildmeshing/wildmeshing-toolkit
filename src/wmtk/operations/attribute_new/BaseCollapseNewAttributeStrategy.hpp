#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "CollapseNewAttributeTopoInfo.hpp"
#include "NewAttributeStrategy.hpp"

namespace wmtk::operations {
// default operation types, default specifies for rational/double we use averages , o/w copytuple
enum class CollapseBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Throw,
    None
};

// This is necessary because subclass is templated
class BaseCollapseNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(const ReturnData& ret_data, const OperationTupleData& tuples) = 0;
};
} // namespace wmtk::operations
