#pragma once
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include "EdgeOperationData.hpp"

namespace wmtk::operations::tri_mesh {
class CollapseNewAttributeStrategy : public wmtk::operations::CollapseNewAttributeStrategy
{
public:
    // NOTE: this constructor doesn't actually use the mesh passed in - this is just to make
    // sure
    // the input mesh is a TriMesh / avoid accidental mistakes
    CollapseNewAttributeStrategy(TriMesh&);


    TriMesh& tri_mesh();
    const TriMesh& tri_mesh() const;

private:
    // the sipmlices that were merged together
    std::vector<std::array<Tuple, 2>> merged_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const final override;

    // the simplices that were created by merging simplices
    std::vector<Tuple> new_simplices(
        const ReturnVariant& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const final override;

    // the sipmlices that were merged together
    std::vector<std::array<Tuple, 2>> merged_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const;

    // the simplices that were created by merging simplices
    std::vector<Tuple> new_simplices(
        const EdgeOperationData& ret_data,
        const Tuple& input_tuple,
        PrimitiveType pt) const;
};
} // namespace wmtk::operations::tri_mesh
