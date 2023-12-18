#include "CollapseNewAttributeStrategy.hpp"


namespace wmtk::operations::tri_mesh {


CollapseNewAttributeStrategy::CollapseNewAttributeStrategy(TriMesh&) {}
TriMesh& CollapseNewAttributeStrategy::tri_mesh()
{
    return static_cast<TriMesh&>(mesh());
}
const TriMesh& CollapseNewAttributeStrategy::tri_mesh() const
{
    return static_cast<const TriMesh&>(mesh());
}

//
// void CollapseNewAttributeStrategy::update_neighboring_simplices(
//    const ReturnVariant& ret_data,
//    PrimitiveType pt,
//    const std::vector<Tuple>& output_simplex) const
//{
//    // default  impl is to do nothing
//}

std::vector<std::array<Tuple, 2>> CollapseNewAttributeStrategy::merged_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return merged_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

// the simplices that were created by merging simplices
std::vector<Tuple> CollapseNewAttributeStrategy::new_simplices(
    const ReturnVariant& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    return new_simplices(std::get<EdgeOperationData>(ret_data), output_tuple, pt);
}

// the sipmlices that were merged together
std::vector<std::array<Tuple, 2>> CollapseNewAttributeStrategy::merged_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    const auto& mesh = this->tri_mesh();
    return mesh.parent_scope([&]() -> std::vector<std::array<Tuple, 2>> {
        switch (get_primitive_type_id(pt)) {
        case 0: {
            return {ret_data.input_endpoints(mesh)};
        }
        case 1: {
            return ret_data.ear_edges(mesh);
        }
        default: return {};
        }
    });
}

// the simplices that were created by merging simplices
std::vector<Tuple> CollapseNewAttributeStrategy::new_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    const auto& mesh = this->tri_mesh();
    switch (get_primitive_type_id(pt)) {
    case 0: {
        return {output_tuple};
    }
    case 1: {
        return ret_data.collapse_merged_ear_edges(mesh);
    }
    default: return {};
    }
}

} // namespace wmtk::operations::tri_mesh
