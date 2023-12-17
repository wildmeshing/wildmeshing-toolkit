#include <wmtk/multimesh/operations/CollapseReturnData.hpp>

#include <wmtk/attribute/SmartAttributeHandle.hpp>

namespace wmtk::multimesh::attribute {

class CollapseNewValueApplicatorBase
{
    using ReturnData = wmtk::multiesh::operations::CollapseReturnData;

public:
    virtual void update_merged_simplices(
        const ReturnData& ret_data,
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& output_simplex) const = 0;
    virtual void update_neighboring_simplices(
        const ReturnData& ret_data,
        PrimitiveType pt,
        const std::vector<Tuple>& output_simplex) const;

    virtual const Mesh& mesh() const = 0;

private:
    // the edge that was collapsed upon
    Tuple collapsed_edge(const ReturnData& ret_data, const Tuple& input_tuple) const;


    // the sipmlices that were merged together
    std::vector<std::array<Tuple, 2>>
    merged_simplices(const ReturnData& ret_data, const Tuple& input_tuple, PrimitiveType pt) const;

    // the simplices that were created by merging simplices
    std::vector<Tuple>
    new_simplices(const ReturnData& ret_data, const Tuple& input_tuple, PrimitiveType pt) const;

    // set of faces whose one ring were modified
    // SHOULD be safe to resurrect to a previous state
    std::vector<Tuple> output_modified_simplices(
        const ReturnData& ret_data,
        const Tuple& output_tuple,
        const PrimitiveType pt) const;


    // the top dimension that were removed in the operation
    // not necessarily used, but defines a unique ordering for
    // * input_ears
    // * merged_edges
    // that defines a correspondence between the two
    std::vector<Tuple> removed_top_dimension_simplices(
        const ReturnData& ret_data,
        const Tuple& input_tuple) const;
};
template <typename T>

SmartAttributeHandle<T>
} // namespace wmtk::multimesh::attribute
