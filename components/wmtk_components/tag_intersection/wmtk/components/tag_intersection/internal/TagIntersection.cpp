#include "TagIntersection.hpp"

namespace wmtk {
namespace components {

bool TagIntersection::simplex_is_in_intersection(
    Mesh& m,
    const simplex::Simplex& s,
    const std::deque<TagAttribute>& input_tag_attributes)
{
    const simplex::SimplexCollection os = simplex::open_star(m, s);

    std::vector<bool> tag_is_present(input_tag_attributes.size(), false);

    for (const simplex::Simplex& s : os.simplex_vector()) {
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


void TagIntersection::compute_intersection(
    Mesh& m,
    const std::vector<std::tuple<attribute::MeshAttributeHandle, int64_t>>& input_tags,
    const std::vector<std::tuple<attribute::MeshAttributeHandle, int64_t>>& output_tags)
{
    std::deque<TagAttribute> input_tag_attributes;
    for (const auto& [handle, val] : input_tags) {
        input_tag_attributes.emplace_back(m, handle, handle.primitive_type(), val);
    }
    std::deque<TagAttribute> output_tag_attributes;
    for (const auto& [handle, val] : output_tags) {
        output_tag_attributes.emplace_back(m, handle, handle.primitive_type(), val);
    }

    // iterate through all simplices and check if the simplex itself or any of its cofaces is
    // tagged
    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
        const simplex::Simplex v = simplex::Simplex::vertex(t);
        if (simplex_is_in_intersection(m, v, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, v);
            }
        }
    }

    if (m.top_cell_dimension() < 1) {
        return;
    }

    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        const simplex::Simplex e = simplex::Simplex::edge(t);
        if (simplex_is_in_intersection(m, e, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, e);
            }
        }
    }

    if (m.top_cell_dimension() < 2) {
        return;
    }

    for (const Tuple& t : m.get_all(PrimitiveType::Face)) {
        const simplex::Simplex f = simplex::Simplex::face(t);
        if (simplex_is_in_intersection(m, f, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, f);
            }
        }
    }

    if (m.top_cell_dimension() < 3) {
        return;
    }

    for (const Tuple& t : m.get_all(PrimitiveType::Tetrahedron)) {
        const simplex::Simplex f = simplex::Simplex::tetrahedron(t);
        if (simplex_is_in_intersection(m, f, input_tag_attributes)) {
            for (TagAttribute& ta : output_tag_attributes) {
                ta.set_tag(m, f);
            }
        }
    }
}

} // namespace components
} // namespace wmtk
