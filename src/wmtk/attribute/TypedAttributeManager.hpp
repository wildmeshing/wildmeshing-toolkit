#pragma once

#include "AttributeHandle.hpp"
#include "CachingAttribute.hpp"


#include <Eigen/Core>

#include <map>
#include <vector>


namespace wmtk {

class MeshWriter;
class Mesh;
namespace attribute {

/**
 * Contains all attributes of type T for a single mesh.
 * It also stores a map so that attributes can be accessed through a name.
 */
template <typename T>
class TypedAttributeManager
{
    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;


public:
    TypedAttributeManager() = default;
    TypedAttributeManager(const TypedAttributeManager& o) = delete;
    TypedAttributeManager(TypedAttributeManager&& o) = default;
    TypedAttributeManager& operator=(const TypedAttributeManager& o) = delete;
    TypedAttributeManager& operator=(TypedAttributeManager&& o) = default;

    void serialize(const int dim, MeshWriter& writer) const;

    [[nodiscard]] AttributeHandle register_attribute(
        const std::string& name,
        int64_t dimension,
        bool replace = false,
        T default_value = T(0));

    // Number of (vector-)values available to be written to, which can be more than the number of
    // simplcies the mesh has
    int64_t reserved_size() const;

    // sets teh size of teh store to be the specified size
    void reserve(const int64_t size);

    // adds size more simplices to teh existing reservation
    void reserve_more(int64_t size);
    // makes sure we have at least size simplices reserved
    void guarantee_at_least(int64_t size);

    /**
     * @brief Remove all passed in attributes.
     *
     * @param attributes Vector of attributes that should be removed.
     * @param invalidate_handles invalidates all handles. If true this garbage collects old handles
     */
    void remove_attributes(const std::vector<AttributeHandle>& attributes);
    /**
     * @brief Remove a single attribute
     *
     * @param attribute the attribute being deleted
     */
    void remove_attribute(const AttributeHandle& attribute);


    bool operator==(const TypedAttributeManager<T>& other) const;
    void push_scope();
    void pop_scope(bool apply_updates = true);
    void rollback_current_scope();

    void change_to_parent_scope() const;
    void change_to_child_scope() const;


    int64_t dimension(const AttributeHandle& handle) const;
    const T& default_value(const AttributeHandle& handle) const;
    std::string get_name(const AttributeHandle& handle) const;

    void set_name(const AttributeHandle& handle, const std::string& name);

    bool has_attribute(const std::string& name) const;

    // the number of active attributes held in this object
    // Note that the set of active attribute indices is not defined by the integers between 0,
    // attribute_count. To get a list of valid handles use active_attributes This function is not
    // that fast
    size_t attribute_count() const;

    // Returns a vector of handles to the set of active attributes
    std::vector<AttributeHandle> active_attributes() const;
    void assert_capacity_valid(int64_t cap) const;

    CachingAttribute<T>& attribute(const AttributeHandle& handle);
    const CachingAttribute<T>& attribute(const AttributeHandle& handle) const;

    AttributeHandle attribute_handle(const std::string& name) const;

    bool is_active(const AttributeHandle& handle) const;

    // pass by value due to
    //https://clang.llvm.org/extra/clang-tidy/checks/modernize/pass-by-value.html
    void set(const AttributeHandle& handle, std::vector<T> val);

    bool validate_handle(const AttributeHandle& handle) const;

protected:
    /// Clears and compactifies the attribute list. This invalidates all existing handles
    [[deprecated]] void clear_dead_attributes();


    size_t attribute_size(const AttributeHandle& handle) const;

private:
    // The vector held in each Attribute in m_attributes has this size
    int64_t m_reserved_size = -1;

    std::vector<std::unique_ptr<CachingAttribute<T>>> m_attributes;
};
template <typename T>
inline CachingAttribute<T>& TypedAttributeManager<T>::attribute(const AttributeHandle& handle)
{
    CachingAttribute<T>& attr = *m_attributes.at(handle.index());
    return attr;
}
template <typename T>
inline const CachingAttribute<T>& TypedAttributeManager<T>::attribute(
    const AttributeHandle& handle) const
{
    return *m_attributes.at(handle.index());
}

template <typename T>
inline int64_t TypedAttributeManager<T>::dimension(const AttributeHandle& handle) const
{
    return attribute(handle).dimension();
}
template <typename T>
inline const T& TypedAttributeManager<T>::default_value(const AttributeHandle& handle) const
{
    return attribute(handle).default_value();
}
} // namespace attribute
} // namespace wmtk
