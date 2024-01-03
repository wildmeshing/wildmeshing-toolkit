#pragma once

#include <wmtk/multimesh/operations/CollapseReturnData.hpp>

namespace wmtk::operations {
class CollapseNewAttributeTopoInfo
{
public:
    using ReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using ReturnVariant = ReturnData::ReturnVariant;

    virtual ~CollapseNewAttributeTopoInfo() = default;

    // these virtuals are handled by per-mesh dimension code
    // the sipmlices that were merged together
    virtual std::vector<std::array<Tuple, 2>> merged_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const = 0;

    // these virtuals are handled by per-mesh dimension code
    // the simplices that were created by merging simplices
    virtual std::vector<Tuple> new_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const = 0;

    // set of faces whose one ring were modified
    // SHOULD be safe to resurrect to a previous state
    // std::vector<Tuple> output_modified_simplices(
    //    const ReturnVariant& ret_data,
    //    PrimitiveType pt,
    //    const Tuple& output_tuple) const = 0;
};
} // namespace wmtk::operations