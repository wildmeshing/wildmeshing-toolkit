#pragma once
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "NewAttributeStrategy.hpp"


namespace wmtk::operations {

class SplitNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::SplitReturnData;
    using ReturnVariant = ReturnData::ReturnVariant;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;
    void update(const ReturnData& ret_data, const OperationTupleData& op_data);

    virtual void update_handle_mesh(Mesh& m) = 0;
    virtual void set_standard_split_rib_strategy(SplitRibBasicStrategy t) = 0;
    virtual void set_standard_split_strategy(SplitBasicStrategy t) = 0;

protected:
    virtual void assign_split(
        PrimitiveType pt,
        const Tuple& input_simplex,
        const std::array<Tuple, 2>& split_simplices) = 0;


    virtual void assign_split_ribs(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_ears,
        const Tuple& final_simplex) = 0;

protected:
    virtual std::vector<std::array<Tuple, 2>> input_ear_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const = 0;

    // the simplices at the boundary of the pairs of simplices that were split
    // vertex should return 0
    // edge should return 2
    virtual std::vector<Tuple> output_rib_simplices(
        const ReturnVariant& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const = 0;


    // the sipmlices that were split from one simplex above
    // vertex should return 1
    virtual std::vector<std::array<Tuple, 2>> output_split_simplices(
        const ReturnVariant& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const = 0;

    // the simplices that were split in half
    // vertex should return 0
    // edge should return 1 (the input edge)
    virtual std::vector<Tuple> input_split_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const = 0;


    //// set of faces whose one ring were modified
    //// SHOULD be safe to resurrect to a previous state
    // virtual std::vector<Tuple> output_modified_simplices(
    //     const ReturnVariant& ret_data,
    //     const PrimitiveType pt,
    //     const Tuple& output_tuple) const = 0;


    //// the top dimension that were removed in the operation
    //// not necessarily used, but defines a unique ordering for
    //// * input_ears
    //// *
    //// that defines a correspondence between the two
    // std::vector<Tuple> split_top_dimension_simplices(
    //     const ReturnVariant& ret_data,
    //     const Tuple& input_tuple) const;
};


} // namespace wmtk::operations
