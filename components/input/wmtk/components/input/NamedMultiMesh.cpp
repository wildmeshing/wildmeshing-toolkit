#include "NamedMultiMesh.hpp"
#include <nlohmann/json.hpp>
#include <vector>
namespace wmtk::components::input {

struct NamedMultiMesh::Node
{
    std::string name;
    std::vector<std::unique_ptr<Node>> m_children;
    void set_names(const nlohmann::json& js)
    {
        if (js.is_null()) {
            return;
        }
        if (js.is_string()) {
            m_children.emplace_back()->name = js;
        } else if (js.is_array()) {
            for (const auto& value : js) {
                auto& child = m_children.emplace_back();
                child->name = value;
            }
        } else if (js.is_object()) {
            for (const auto& [key, value] : js.items()) {
                auto& child = m_children.emplace_back();
                child->name = key;
                child->set_names(value);
            }
        }
    }
};
void NamedMultiMesh::set_names(const std::string_view& root_name)
{
    set_names("", {});
}
void NamedMultiMesh::set_names(const std::string_view& root_name, const nlohmann::json& js)
{
    m_name_root = std::make_unique<Node>();
    m_name_root->name = root_name;
    if (js.is_null()) {
        return;
    }
    if (js.is_object()) {
        m_name_root->set_names(js);
    }
}

NamedMultiMesh::NamedMultiMesh() = default;
NamedMultiMesh::~NamedMultiMesh() = default;
} // namespace wmtk::components::input
