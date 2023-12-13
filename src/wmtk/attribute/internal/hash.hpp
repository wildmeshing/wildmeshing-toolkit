#pragma once
#include <functional>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/utils/vector_hash.hpp>


template <>
struct std::hash<wmtk::attribute::AttributeHandle>
{
    size_t operator()(const wmtk::attribute::AttributeHandle& handle) const noexcept;
};
template <>
template <typename T>
struct std::hash<wmtk::attribute::MeshAttributeHandle<T>>
{
    size_t operator()(const wmtk::attribute::MeshAttributeHandle<T>& handle) const noexcept
    {
        std::vector<size_t> data;
        data.emplace_back(std::hash<wmtk::attribute::AttributeHandle>{}(handle.m_base_handle));
        data.emplace_back(wmtk::get_primitive_type_id(handle.primitive_type()));
        return wmtk::utils::vector_hash(data);
    }
};
