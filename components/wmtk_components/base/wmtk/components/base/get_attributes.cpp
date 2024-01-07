#include "get_attributes.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::components::base {

std::vector<attribute::MeshAttributeHandle> get_attributes(
    const Mesh& m,
    const std::vector<std::string>& attribute_names)
{
    std::vector<attribute::MeshAttributeHandle> handles;

    for (const std::string& name : attribute_names) {
        int64_t count = 0;

        for (const PrimitiveType& ptype : utils::primitive_below(m.top_simplex_type())) {
            if (m.has_attribute<char>(name, ptype)) {
                ++count;
                handles.emplace_back(m.get_attribute_handle<char>(name, ptype));
            }
            if (m.has_attribute<int64_t>(name, ptype)) {
                ++count;
                handles.emplace_back(m.get_attribute_handle<int64_t>(name, ptype));
            }
            if (m.has_attribute<double>(name, ptype)) {
                ++count;
                handles.emplace_back(m.get_attribute_handle<double>(name, ptype));
            }
            if (m.has_attribute<Rational>(name, ptype)) {
                ++count;
                handles.emplace_back(m.get_attribute_handle<Rational>(name, ptype));
            }
        }

        if (count == 0) {
            log_and_throw_error("Attribute with name {} was not found.");
        }
        if (count > 1) {
            log_and_throw_error(
                "Attribute name ambiguity: {} attributes have the name {}",
                count,
                name);
        }
    }

    return handles;
}

} // namespace wmtk::components::base