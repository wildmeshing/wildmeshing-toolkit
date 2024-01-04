#pragma once
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include "EdgeOperationData.hpp"

namespace wmtk::operations::tri_mesh {
class CollapseNewAttributeTopoInfo : public wmtk::operations::CollapseNewAttributeTopoInfo
{
public:
    CollapseNewAttributeTopoInfo(TriMesh& m);

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

private:
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

    TriMesh& m_mesh;
};
} // namespace wmtk::operations::tri_mesh
