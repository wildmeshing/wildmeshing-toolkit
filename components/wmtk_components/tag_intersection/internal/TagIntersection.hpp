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
class TagAttribute
{
public:
    Accessor<long> m_accessor;
    PrimitiveType m_ptype;
    long m_val;

    TagAttribute(Mesh& m, const MeshAttributeHandle<long>& attribute, PrimitiveType ptype, long val)
        : m_accessor(m.create_accessor(attribute))
        , m_ptype(ptype)
        , m_val(val)
    {}

    TagAttribute(TagAttribute&) = delete;

    bool is_tagged(Mesh& m, const simplex::Simplex& s) const
    {
        if (s.primitive_type() != m_ptype) {
            return false;
        }
        return m_accessor.scalar_attribute(s.tuple()) == m_val;
    }

    void set_tag(Mesh& m, const simplex::Simplex& s)
    {
        if (s.primitive_type() != m_ptype) {
            return;
        }
        m_accessor.scalar_attribute(s.tuple()) = m_val;
    }
};

class TagIntersection
{
public:
    void compute_intersection(
        TriMesh& m,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& input_tags,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& output_tags);

    void compute_intersection(
        TetMesh& m,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& input_tags,
        const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& output_tags);

private:
    bool simplex_is_in_intersection(
        Mesh& m,
        const Simplex& v,
        const std::deque<TagAttribute>& input_tag_attributes);
};

} // namespace components
} // namespace wmtk
