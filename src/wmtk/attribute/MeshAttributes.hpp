#pragma once

#include <wmtk/utils/MerkleTreeInteriorNode.hpp>
#include "Attribute.hpp"
#include "AttributeHandle.hpp"


#include <Eigen/Core>

#include <map>
#include <vector>


// TODO: this is just a fancy vector for attributes, perhaps this can be recycled / simplified. The reserved quantiy should be held by a level above this abstraction (as multiple types should all have hte same reservation size)

namespace wmtk {

namespace io {
class MeshWriter;
}
class Mesh;
namespace attribute {
template <typename T, int Dim>
class AccessorBase;

/**
 * Contains all attributes of type T for a single mesh.
 * It also stores a map so that attributes can be accessed through a name.
 */
template <typename T>
class MeshAttributes : public wmtk::utils::MerkleTreeInteriorNode
{
    template <typename U, int D>
    friend class AccessorBase;

    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;


public:
    MeshAttributes() = default;
    MeshAttributes(const MeshAttributes& o) = delete;
    MeshAttributes(MeshAttributes&& o) = default;
    MeshAttributes& operator=(const MeshAttributes& o) = delete;
    MeshAttributes& operator=(MeshAttributes&& o) = default;

    void serialize(const int dim, io::MeshWriter& writer) const;

    // attribute directly hashes its "child_hashables" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    [[nodiscard]] AttributeHandle register_attribute(
        const std::string& name,
        int64_t dimension,
        bool replace = false,
        T default_value = T(0));

    int64_t reserved_size() const;
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
    void remove_attributes(
        const std::vector<AttributeHandle>& attributes,
        bool invalidate_handles = true);
    /**
     * @brief Remove a single attribute
     *
     * @param attribute the attribute being deleted
     */
    void remove_attribute(const AttributeHandle& attribute);


    bool operator==(const MeshAttributes<T>& other) const;
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

    Attribute<T>& attribute(const AttributeHandle& handle);
    const Attribute<T>& attribute(const AttributeHandle& handle) const;

    AttributeHandle attribute_handle(const std::string& name) const;

    bool is_active(const AttributeHandle& handle) const;

    // pass by value due to
    //https://clang.llvm.org/extra/clang-tidy/checks/modernize/pass-by-value.html
    void set(const AttributeHandle& handle, std::vector<T> val);

protected:
    /// Clears and compactifies the attribute list. This invalidates all existing handles
    void clear_dead_attributes();


    size_t attribute_size(const AttributeHandle& handle) const;

private:
    std::map<std::string, AttributeHandle> m_handles;

    // The vector held in each Attribute in m_attributes has this size
    int64_t m_reserved_size = -1;

    std::vector<std::unique_ptr<Attribute<T>>> m_attributes;
};
template <typename T>
inline Attribute<T>& MeshAttributes<T>::attribute(const AttributeHandle& handle)
{
    Attribute<T>& attr = *m_attributes.at(handle.index);
    return attr;
}
template <typename T>
inline const Attribute<T>& MeshAttributes<T>::attribute(const AttributeHandle& handle) const
{
    return *m_attributes.at(handle.index);
}

template <typename T>
inline int64_t MeshAttributes<T>::dimension(const AttributeHandle& handle) const
{
    return attribute(handle).dimension();
}
template <typename T>
inline const T& MeshAttributes<T>::default_value(const AttributeHandle& handle) const
{
    return attribute(handle).default_value();
}
} // namespace attribute
} // namespace wmtk
