#include "NamedMultiMesh.hpp"
#include <nlohmann/json.hpp>
#include <vector>
namespace wmtk::components::input {

struct NamedMultiMesh::Node
{
    std::string name;
    std::vector<std::unique_ptr<Node>> m_children;
    void set_names(const nlohmann::json& js) {}
};
void NamedMultiMesh::set_names(const nlohmann::json& js, const std::string_view& name)
{
    m_name_root = std::make_unique<Node>();
    m_name_root->name = name;
    m_name_root->set_names(js);
}

NamedMultiMesh::NamedMultiMesh() = default;
NamedMultiMesh::~NamedMultiMesh() = default;
} // namespace wmtk::components::input
