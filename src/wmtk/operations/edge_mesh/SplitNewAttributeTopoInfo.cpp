
#include "SplitNewAttributeTopoInfo.hpp"

namespace wmtk::operations::edge_mesh {

SplitNewAttributeTopoInfo::SplitNewAttributeTopoInfo(EdgeMesh& m)
    : m_mesh(m)
{}

//
// void SplitNewAttributeTopoInfo::update_neighboring_simplices(
//    const ReturnVariant& ret_data,
//    PrimitiveType pt,
//    const std::vector<Tuple>& output_simplex) const
//{
//    // default  impl is to do nothing
//}

std::vector<std::array<Tuple, 2>> SplitNewAttributeTopoInfo::input_ear_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return input_ear_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

// the simplices that were created by merging simplices
std::vector<Tuple> SplitNewAttributeTopoInfo::output_rib_simplices(
    const ReturnVariant& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    return output_rib_simplices(std::get<EdgeOperationData>(ret_data), output_tuple, pt);
}
std::vector<std::array<Tuple, 2>> SplitNewAttributeTopoInfo::output_split_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return output_split_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

// the simplices that were created by merging simplices
std::vector<Tuple> SplitNewAttributeTopoInfo::input_split_simplices(
    const ReturnVariant& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return input_split_simplices(std::get<EdgeOperationData>(ret_data), input_tuple, pt);
}

std::vector<std::array<int64_t, 2>> SplitNewAttributeTopoInfo::output_duplicated_free_simplices(
    const ReturnVariant& ret_data,
    PrimitiveType pt) const
{
    return output_duplicated_free_simplices(std::get<EdgeOperationData>(ret_data), pt);
}

std::vector<std::array<Tuple, 2>> SplitNewAttributeTopoInfo::output_split_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    int64_t id = get_primitive_type_id(pt);

    switch (id) {
    case 0: {
        return {};
    }
    case 1: {
        return {ret_data.split_output_edges(m_mesh)};
    }
    default: return {};
    }
}

std::vector<Tuple> SplitNewAttributeTopoInfo::input_split_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    switch (get_primitive_type_id(pt)) {
    case 1: {
        return {input_tuple};
    }
    default: return {};
    }
}

std::vector<std::array<Tuple, 2>> SplitNewAttributeTopoInfo::input_ear_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& input_tuple,
    PrimitiveType pt) const
{
    return m_mesh.parent_scope([&]() -> std::vector<std::array<Tuple, 2>> {
        switch (get_primitive_type_id(pt)) {
        case 0: {
            return {ret_data.input_endpoints(m_mesh)};
        }
        default: return {};
        }
    });
}

// the simplices that were created by merging simplices
std::vector<Tuple> SplitNewAttributeTopoInfo::output_rib_simplices(
    const EdgeOperationData& ret_data,
    const Tuple& output_tuple,
    PrimitiveType pt) const
{
    switch (get_primitive_type_id(pt)) {
    case 0: {
        return {output_tuple};
    }
    default: return {};
    }
}

std::vector<std::array<int64_t, 2>> SplitNewAttributeTopoInfo::output_duplicated_free_simplices(
    const EdgeOperationData& ret_data,
    PrimitiveType pt) const
{
    assert(m_mesh.is_free());
    switch (get_primitive_type_id(pt)) {
    case 0: {
        return {ret_data.m_free_split_v};
    }
    case 1: {
        return {};
    }
    default: return {};
    }
}

} // namespace wmtk::operations::edge_mesh
