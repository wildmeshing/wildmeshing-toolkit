#pragma once

#include <wmtk/utils/MerkleTreeInteriorNode.hpp>
#include "Attribute.hpp"
#include "AttributeHandle.hpp"


#include <Eigen/Core>

#include <map>
#include <vector>


namespace wmtk {

class MeshWriter;
class Mesh;
namespace attribute {
template <typename T>
class AccessorBase;

/**
 * Contains all attributes of type T for a single mesh.
 * It also stores a map so that attributes can be accessed through a name.
 */
template <typename T>
class MeshAttributes : public wmtk::utils::MerkleTreeInteriorNode
{
    friend class AccessorBase<T>;
    friend class wmtk::Mesh;

    typedef Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> MapResult;
    typedef Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>> ConstMapResult;


public:
    MeshAttributes();
    MeshAttributes(const MeshAttributes& o);
    MeshAttributes(MeshAttributes&& o);
    MeshAttributes& operator=(const MeshAttributes& o);
    MeshAttributes& operator=(MeshAttributes&& o);

    void serialize(const int dim, MeshWriter& writer) const;

    // attribute directly hashes its "child_hashables" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    [[nodiscard]] AttributeHandle register_attribute(
        const std::string& name,
        long dimension,
        bool replace = false,
        T default_value = T(0));

    long reserved_size() const;
    void reserve(const long size);

    void reserve_more(long size);

    void clear_attributes(const std::vector<AttributeHandle>& keep_attributes);

    bool operator==(const MeshAttributes<T>& other) const;
    void push_scope();
    void pop_scope(bool apply_updates = true);
    void clear_current_scope();

    void change_to_parent_scope() const;
    void change_to_leaf_scope() const;


    long dimension(const AttributeHandle& handle) const;
    std::string get_name(const AttributeHandle& handle) const;

    bool has_attribute(const std::string& name) const;

protected:
    AttributeHandle attribute_handle(const std::string& name) const;


    Attribute<T>& attribute(const AttributeHandle& handle);
    const Attribute<T>& attribute(const AttributeHandle& handle) const;

    // pass by value due to
    //https://clang.llvm.org/extra/clang-tidy/checks/modernize/pass-by-value.html
    void set(const AttributeHandle& handle, std::vector<T> val);

    size_t attribute_size(const AttributeHandle& handle) const;

private:
    std::map<std::string, AttributeHandle> m_handles;

    // The vector held in each Attribute in m_attributes has this size
    long m_reserved_size = -1;

    std::vector<Attribute<T>> m_attributes;
};
} // namespace attribute
} // namespace wmtk
