#pragma once

#include <deque>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {
namespace components {
/**
 * @name TagAttribute
 * @brief this class is used to manage different tags among the different primitivetypes.
 */
class TagAttribute
{
public:
    Accessor<long> m_tag_accessor; // access primitives' tag value
    PrimitiveType m_ptype; // record this tag attribute's primitivetype
    long m_tag_val; // a given tagvalue used to check or tag the attributes

    TagAttribute(Mesh& m, const MeshAttributeHandle<long>& attribute, PrimitiveType ptype, long val)
        : m_tag_accessor(m.create_accessor(attribute))
        , m_ptype(ptype)
        , m_tag_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;

    /**
     * @name is_tagged
     * @brief if a given simplex is not the current attributehandle's primitivetype, return false.
     * otherwise check if a given simplex's attribute is tagged, if yes return true, else return
     * false.
     * @param m mesh
     * @param s simplex
     */
    bool is_tagged(Mesh& m, const simplex::Simplex& s) const
    {
        if (s.primitive_type() != m_ptype) {
            return false;
        }
        return m_tag_accessor.scalar_attribute(s.tuple()) == m_tag_val;
    }

    /**
     * @name set_tag
     * @brief first check if the given simplex's primitivetype is equal to the attibutehandle's. if
     * yes, then tag this simplex with the given tag_val, otherwise do nothing.
     * @param m mesh
     * @param s simplex
     */
    void set_tag(Mesh& m, const simplex::Simplex& s)
    {
        if (s.primitive_type() != m_ptype) {
            return;
        }
        m_tag_accessor.scalar_attribute(s.tuple()) = m_tag_val;
    }
};

/**
 * @name TagIntersection
 * @brief this class is used to compute the couple primitives' tags' intersection when one wants to
 * get a intersection tag attribute between several tag attributes.
 */
class TagIntersection
{
public:
    /**
     * @name compute_intersection
     * @param m trimesh
     * @param input_tags a vector that contain different tag attributes and the tag value which
     * needs to be considered
     * @param output_tags a vector of attibutehandles, which contains the intersection tags
     */
    void compute_intersection(
        TriMesh& m,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& input_tags,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& output_tags);
    /**
     * @name compute_intersection
     * @param m tetmesh
     * @param input_tags a vector that contain different tag attributes and the tag value which
     * needs to be considered
     * @param output_tags a vector of attibutehandles, which contains the intersection tags
     */
    void compute_intersection(
        TetMesh& m,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& input_tags,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& output_tags);

private:
    /**
     * @name simplex_is_in_intersection
     * @brief this function is invoked when we check if a given simplex is an intersection
     * simplex(tagged by all tag attributes), we will check the given simplex's one-ring. If in the
     * region of one-ring, the given simplex is tagged by all attributes, then return true,
     * otherwise return false.
     * @param m mesh
     * @param s a given simplex
     * @param input_tag_attributes a given tag attribute needs to be considered to compute the
     * tag intersection
     */
    bool simplex_is_in_intersection(
        Mesh& m,
        const Simplex& s,
        const std::deque<TagAttribute>& input_tag_attributes);
};

} // namespace components
} // namespace wmtk
