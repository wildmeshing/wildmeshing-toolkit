#pragma once
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "NewAttributeStrategy.hpp"
#include "SplitNewAttributeTopoInfo.hpp"


namespace wmtk::operations {

// default operation types
enum class SplitBasicStrategy { Default, Copy, Half, Throw, None };
//rib and collapse have hte same prototypes / default funs available
enum class SplitRibBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Throw,
    None
};

// This is necessary because subclass is templated
class BaseSplitNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::SplitReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(const ReturnData& ret_data, const OperationTupleData& op_data) = 0;
};


} // namespace wmtk::operations
