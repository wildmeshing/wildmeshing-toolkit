
#include "SplitNewAttributeStrategy.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::operations::tri_mesh {
SplitNewAttributeStrategy::SplitNewAttributeStrategy(TriMesh&) {}
TriMesh& SplitNewAttributeStrategy::tri_mesh()
{
    return static_cast<TriMesh&>(mesh());
}
const TriMesh& SplitNewAttributeStrategy::tri_mesh() const
{
    return static_cast<const TriMesh&>(mesh());
}

//
// void SplitNewAttributeStrategy::update_neighboring_simplices(
//    const ReturnVariant& ret_data,
//    PrimitiveType pt,
//    const std::vector<Tuple>& output_simplex) const
//{
//    // default  impl is to do nothing
//}

std::vector<std::array<Tuple, 2>> SplitNewAttributeStrategy::input_ear_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return input_ear_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

// the simplices that were created by merging simplices
std::vector<Tuple> SplitNewAttributeStrategy::output_rib_simplices(
    const ReturnVariant& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    return output_rib_simplices(std::get<EdgeOperationData>(ret_data), output_tuple, pt);
}
std::vector<std::array<Tuple, 2>> SplitNewAttributeStrategy::output_split_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return output_split_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

// the simplices that were created by merging simplices
std::vector<Tuple> SplitNewAttributeStrategy::input_split_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return input_split_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}


std::vector<std::array<Tuple, 2>> SplitNewAttributeStrategy::output_split_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    const auto& mesh = this->tri_mesh();
    int64_t id = get_primitive_type_id(pt);
    switch (id) {
    case 0: {
        return {};
    }
    case 1: {
        return {ret_data.split_output_edges(mesh)};
    }
    case 2: {
        return ret_data.split_output_faces(mesh);
    }
    default: return {};
    }
}

std::vector<Tuple> SplitNewAttributeStrategy::input_split_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    const auto& mesh = this->tri_mesh();
    switch (get_primitive_type_id(pt)) {
    case 0: {
        return {};
    }
    case 1: {
        return {input_tuple};
    }
    case 2: {
        return ret_data.input_faces(mesh);
    }
    default: return {};
    }
}

std::vector<std::array<Tuple, 2>> SplitNewAttributeStrategy::input_ear_simplices(
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
std::vector<Tuple> SplitNewAttributeStrategy::output_rib_simplices(
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
        return ret_data.split_new_rib_edges(mesh);
    }
    default: return {};
    }
}
} // namespace wmtk::operations::tri_mesh
