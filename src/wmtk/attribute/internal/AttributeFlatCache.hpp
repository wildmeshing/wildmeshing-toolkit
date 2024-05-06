#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "AttributeCacheData.hpp"
#include "MapTypes.hpp"


namespace wmtk::attribute {
template <typename T>
class Attribute;
template <typename T, int Dim>
class AccessorBase;
namespace internal {
template <typename T>
class AttributeFlatCache
{
public:
    using Data = AttributeCacheData<T>;

    using MapResult = internal::MapResult<T>;
    using ConstMapResult = internal::ConstMapResult<T>;


    AttributeFlatCache();
    ~AttributeFlatCache();
    AttributeFlatCache(const AttributeFlatCache&) = delete;
    AttributeFlatCache& operator=(const AttributeFlatCache&) = delete;
    AttributeFlatCache(AttributeFlatCache&&) = default;
    AttributeFlatCache& operator=(AttributeFlatCache&&) = default;


    template <typename Derived>
    void try_caching(int64_t index, const Eigen::MatrixBase<Derived>& value);
    void try_caching(int64_t index, const T& value);


    const T* get_value(int64_t index, size_t dim) const;

    void clear();
    size_t size() const { return m_indices.size(); }

    void apply_to(Attribute<T>& attribute) const;
    void apply_to(AttributeFlatCache<T>& other) const;

    // applyes to some other buffer that was passed in
    void apply_to(const Attribute<T>& attribute, std::vector<T>& other) const;

    const std::vector<T>& buffer() const { return m_buffer; }
    const std::vector<std::pair<size_t, size_t>>& indices() const { return m_indices; }

protected:
    std::vector<T> m_buffer;
    std::vector<std::pair<size_t, size_t>> m_indices;
};

} // namespace internal
} // namespace wmtk::attribute

#include "AttributeFlatCache.hxx"
