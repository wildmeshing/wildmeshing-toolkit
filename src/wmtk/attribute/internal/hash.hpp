#pragma once
#include <functional>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/utils/vector_hash.hpp>


// in general we're making classes implement the hashable interface - but these
// handles are litered around as simple objects so we're not enforcing that.
// Instead we're using std::hash as a visitor

namespace wmtk {
template <typename T>
class hash
{
};
template <>
class hash<wmtk::attribute::AttributeHandle>
{
public:
    size_t operator()(const wmtk::attribute::AttributeHandle& handle) const noexcept;
};
template <typename T>
class hash<wmtk::attribute::TypedAttributeHandle<T>>
{
public:
    size_t operator()(const wmtk::attribute::TypedAttributeHandle<T>& handle) const noexcept
    {
        std::vector<size_t> data;
        data.emplace_back(handle_hash(handle));
        data.emplace_back(primitive_hash(handle));
        return wmtk::utils::vector_hash(data);
    }
    size_t handle_hash(const wmtk::attribute::TypedAttributeHandle<T>& handle) const noexcept
    {
        return hash<wmtk::attribute::AttributeHandle>{}(handle.m_base_handle);
    }
    size_t primitive_hash(const wmtk::attribute::TypedAttributeHandle<T>& handle) const noexcept
    {
        return wmtk::get_primitive_type_id(handle.primitive_type());
    }
};
template <typename T>
class hash<wmtk::attribute::MeshAttributeHandle<T>>
{
public:
    size_t operator()(const wmtk::attribute::MeshAttributeHandle<T>& handle) const noexcept
    {
        std::vector<size_t> data;
        data.emplace_back(handle_hash(handle));
        data.emplace_back(mesh_hash(handle));
        return wmtk::utils::vector_hash(data);
    }
    size_t handle_hash(const wmtk::attribute::MeshAttributeHandle<T>& handle) const noexcept
    {
        return hash<wmtk::attribute::TypedAttributeHandle<T>>{}(handle);
    }
    size_t mesh_hash(const wmtk::attribute::MeshAttributeHandle<T>& handle) const noexcept
    {
        if (!handle.is_valid()) {
            return -1; // TODO: this assumes the vector hash never returns a value of -1
        }
        // here we hash off of the absolute mesh id rather than the mesh itself to prevent cyclic
        // hashing
        return wmtk::utils::vector_hash(handle.mesh().absolute_multi_mesh_id());
    }
};
} // namespace wmtk
template <>
class std::hash<wmtk::attribute::AttributeHandle>
    : public wmtk::hash<wmtk::attribute::AttributeHandle>
{
};
template <>
class std::hash<wmtk::attribute::TypedAttributeHandle<double>>
    : public wmtk::hash<wmtk::attribute::TypedAttributeHandle<double>>
{
};
template <>
class std::hash<wmtk::attribute::TypedAttributeHandle<int64_t>>
    : public wmtk::hash<wmtk::attribute::TypedAttributeHandle<int64_t>>
{
};
template <>
class std::hash<wmtk::attribute::TypedAttributeHandle<char>>
    : public wmtk::hash<wmtk::attribute::TypedAttributeHandle<char>>
{
};
template <>
class std::hash<wmtk::attribute::TypedAttributeHandle<wmtk::Rational>>
    : public wmtk::hash<wmtk::attribute::TypedAttributeHandle<wmtk::Rational>>
{
};
template <>
class std::hash<wmtk::attribute::MeshAttributeHandle<double>>
    : public wmtk::hash<wmtk::attribute::MeshAttributeHandle<double>>
{
};
template <>
class std::hash<wmtk::attribute::MeshAttributeHandle<int64_t>>
    : public wmtk::hash<wmtk::attribute::MeshAttributeHandle<int64_t>>
{
};
template <>
class std::hash<wmtk::attribute::MeshAttributeHandle<char>>
    : public wmtk::hash<wmtk::attribute::MeshAttributeHandle<char>>
{
};
template <>
class std::hash<wmtk::attribute::MeshAttributeHandle<wmtk::Rational>>
    : public wmtk::hash<wmtk::attribute::MeshAttributeHandle<wmtk::Rational>>
{
};
