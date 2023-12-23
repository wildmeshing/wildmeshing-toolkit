#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "NewAttributeStrategy.hpp"

namespace wmtk::operations {
class CollapseNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;
    using ReturnVariant = ReturnData::ReturnVariant;
    void update(const ReturnData& ret_data, const OperationTupleData& tuples);
    virtual void update_handle_mesh(Mesh& m) = 0;

    virtual void set_standard_collapse_strategy(CollapseBasicStrategy t) = 0;

protected:
    virtual void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex) = 0;


protected:
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
