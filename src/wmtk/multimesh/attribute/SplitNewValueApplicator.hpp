
#include <array>
#include <vector>
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
class SplitNewValueApplicatorBase
{
private:
    // the edge that was collapsed upon
    Tuple split_edge(const ReturnData& ret_data, const Tuple& input_tuple) const;

    // the simplices at the boundary of the pairs of simplices that were split
    // vertex should return 0
    // edge should return 2
    std::vector<Tuple> new_spine_simplices(
        const ReturnData& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const;


    // the sipmlices that were split from one
    // vertex should return 1
    std::vector<std::array<Tuple, 2>> new_rib_simplices(
        const ReturnData& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const;

    // the simplices that were split in half
    // vertex should return 0
    // edge should return 1 (the input edge)
    std::vector<Tuple> split_rib_simplices(
        const ReturnData& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const;


    // set of faces whose one ring were modified
    // SHOULD be safe to resurrect to a previous state
    std::vector<Tuple> output_modified_simplices(
        const ReturnData& ret_data,
        const Tuple& output_tuple,
        const PrimitiveType pt) const;


    // the top dimension that were removed in the operation
    // not necessarily used, but defines a unique ordering for
    // * input_ears
    // *
    // that defines a correspondence between the two
    std::vector<Tuple> split_top_dimension_simplices(
        const ReturnData& ret_data,
        const Tuple& input_tuple) const;
};
