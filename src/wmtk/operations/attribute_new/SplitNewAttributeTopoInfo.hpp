namespace wmtk::operations {
class SplitNewAttributeTopoInfo
{
public:
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
