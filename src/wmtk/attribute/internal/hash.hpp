#pragma once
#include <functional>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
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
template <>
class hash<wmtk::attribute::MeshAttributeHandle>
{
public:
    inline size_t operator()(const wmtk::attribute::MeshAttributeHandle& handle) const noexcept;
    inline size_t handle_hash(const wmtk::attribute::MeshAttributeHandle& handle) const noexcept;
    inline size_t mesh_hash(const wmtk::attribute::MeshAttributeHandle& handle) const noexcept;
};
template <int D>
class hash<attribute::utils::HybridRationalAttribute<D>>
{
public:
    size_t operator()(const attribute::utils::HybridRationalAttribute<D>& d) const
    {
        std::vector<size_t> data;
        data.emplace_back(hash<TypedAttributeHandle<char>>{}(d.get_char()));
        data.emplace_back(hash<TypedAttributeHandle<double>>{}(d.get_double()));
        data.emplace_back(hash<TypedAttributeHandle<wmtk::Rational>>{}(d.get_rational()));
        return wmtk::utils::vector_hash(data);
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
class std::hash<wmtk::attribute::MeshAttributeHandle>
    : public wmtk::hash<wmtk::attribute::MeshAttributeHandle>
{
};
template <int D>
class std::hash<wmtk::attribute::utils::HybridRationalAttribute<D>>
    : public wmtk::hash<wmtk::attribute::utils::HybridRationalAttribute<D>>
{
};
