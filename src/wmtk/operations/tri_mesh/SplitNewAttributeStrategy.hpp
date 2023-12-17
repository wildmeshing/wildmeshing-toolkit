#pragma once
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include "EdgeOperationData.hpp"

namespace wmtk::operations::tri_mesh {
class SplitNewAttributeStrategy : public wmtk::operations::SplitNewAttributeStrategy
{
public:
    // NOTE: this constructor doesn't actually use the mesh passed in - this is just to make
    // sure
    // the input mesh is a TriMesh / avoid accidental mistakes
    SplitNewAttributeStrategy(TriMesh&);


    TriMesh& tri_mesh();
    const TriMesh& tri_mesh() const;

private:
    std::vector<std::array<Tuple, 2>> input_ear_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const final override;

    // the simplices at the boundary of the pairs of simplices that were split
    // vertex should return 0
    // edge should return 2
    std::vector<Tuple> output_rib_simplices(
        const ReturnVariant& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const final override;


    // the sipmlices that were split from one simplex above
    // vertex should return 1
    std::vector<std::array<Tuple, 2>> output_split_simplices(
        const ReturnVariant& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const final override;

    // the simplices that were split in half
    // vertex should return 0
    // edge should return 1 (the input edge)
    std::vector<Tuple> input_split_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const final override;

    std::vector<std::array<Tuple, 2>> input_ear_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const;

    // the simplices at the boundary of the pairs of simplices that were split
    // vertex should return 0
    // edge should return 2
    std::vector<Tuple> output_rib_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const;


    // the sipmlices that were split from one simplex above
    // vertex should return 1
    std::vector<std::array<Tuple, 2>> output_split_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& output_tuple,
        PrimitiveType pt) const;

    // the simplices that were split in half
    // vertex should return 0
    // edge should return 1 (the input edge)
    std::vector<Tuple> input_split_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const;
};
} // namespace wmtk::operations::tri_mesh
