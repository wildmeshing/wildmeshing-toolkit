#include "tag_intersection.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/TagIntersectionOptions.hpp"

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

bool simplex_is_in_intersection(
    Mesh& m,
    const Simplex& v,
    const std::deque<TagAttribute>& input_tag_attributes)
{
    const simplex::SimplexCollection os = simplex::open_star(m, v);

    std::vector<bool> tag_is_present(input_tag_attributes.size(), false);

    for (const Simplex& s : os.simplex_vector()) {
        for (size_t i = 0; i < input_tag_attributes.size(); ++i) {
            if (input_tag_attributes[i].is_tagged(m, s)) {
                tag_is_present[i] = true;
            }
        }
    }

    // check if all tags are present
    bool is_intersection = true;
    for (const bool b : tag_is_present) {
        is_intersection = is_intersection && b;
    }
    return is_intersection;
}


void tag_intersection_tri(
    TriMesh& m,
    const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& input_tags,
    const std::vector<std::tuple<MeshAttributeHandle<long>, long>>& output_tags)
{
    std::deque<TagAttribute> input_tag_attributes;
    for (const auto& [handle, val] : input_tags) {
        input_tag_attributes.emplace_back(m, handle, handle.primitive_type(), val);
    }
    std::deque<TagAttribute> output_tag_attributes;
    for (const auto& [handle, val] : output_tags) {
        output_tag_attributes.emplace_back(m, handle, handle.primitive_type(), val);
    }

    // iterate through all simplices and check if the simplex itself or any of its cofaces is tagged
    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const Simplex v = Simplex::vertex(t);
        if (simplex_is_in_intersection(m, v, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, v);
            }
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const Simplex e = Simplex::edge(t);
        if (simplex_is_in_intersection(m, e, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, e);
            }
        }
    }
    for (const Tuple& t : m.get_all(PrimitiveType::Face)) {
        const Simplex f = Simplex::face(t);
        if (simplex_is_in_intersection(m, f, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, f);
            }
        }
    }
}


void tag_intersection(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    TagIntersectionOptions options = j.get<TagIntersectionOptions>();

    auto mesh_in = cache.read_mesh(options.input);


    std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags;
    for (const auto& [name, ptype, val] : options.input_tags) {
        assert(ptype != PrimitiveType::Tetrahedron);

        auto handle = mesh_in->get_attribute_handle<long>(name, ptype);
        input_tags.emplace_back(std::make_tuple(handle, val));
    }

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags;
    for (const auto& [name, ptype, val] : options.output_tags) {
        auto handle = mesh_in->register_attribute<long>(name, ptype, 1);
        output_tags.emplace_back(std::make_tuple(handle, val));
    }

    switch (mesh_in->top_simplex_type()) {
    case PrimitiveType::Face: {
        TriMesh& m = static_cast<TriMesh&>(*mesh_in);
        tag_intersection_tri(m, input_tags, output_tags);
        break;
    }
    default: {
        log_and_throw_error("Works only for triangle meshes: {}", mesh_in->top_simplex_type());
        break;
    }
    }


    cache.write_mesh(*mesh_in, options.output);
}
} // namespace components
} // namespace wmtk
