#include "NamedMultiMesh.hpp"
#include <nlohmann/json.hpp>
#include <ranges>
#include <vector>
#include <wmtk/Mesh.hpp>
namespace wmtk::components::input {
namespace {
auto get_split_path(const std::string_view& view)
{
    using namespace std;
    return std::ranges::views::split(view, "."sv) |
           std::views::transform([](const auto& r) noexcept -> std::string_view {
               return std::string_view(r.begin(), r.end());
           });
}
} // namespace

struct NamedMultiMesh::Node
{
    std::string name;
    std::vector<std::unique_ptr<Node>> m_children;

    std::map<std::string_view, int64_t> m_child_indexer;
    void set_names(const nlohmann::json& js)
    {
        if (js.is_null()) {
            return;
        }
        if (js.is_string()) {
            auto& child = m_children.emplace_back(std::make_unique<Node>());
            child->name = js;
        } else if (js.is_array()) {
            for (const auto& value : js) {
                auto& child = m_children.emplace_back(std::make_unique<Node>());
                child->name = value;
            }
        } else if (js.is_object()) {
            for (const auto& [key, value] : js.items()) {
                auto& child = m_children.emplace_back(std::make_unique<Node>());
                child->name = key;
                child->set_names(value);
            }
        }
        update_child_names();
    }

    void update_child_names()
    {
        m_child_indexer.clear();
        for (size_t j = 0; j < m_children.size(); ++j) {
            m_child_indexer.emplace(m_children[j]->name, j);
        }
    }
};

void NamedMultiMesh::set_root(Mesh& m)
{
    m_root = m.shared_from_this();
}

void NamedMultiMesh::set_mesh(Mesh& m)
{
    set_root(m.get_multi_mesh_root());
}


Mesh& NamedMultiMesh::get_mesh(const std::string_view& path) const
{
    const auto id = get_id(path);
    return m_root->get_multi_mesh_child_mesh(id);
}

auto NamedMultiMesh::get_id(const std::string_view& path) const -> std::vector<int64_t>
{
    std::ranges::view auto split = get_split_path(path);

    std::vector<int64_t> indices;
    Node const* cur_mesh = m_name_root.get();
    assert(*split.begin() == cur_mesh->name || *split.begin() == "");
    for (const auto& token : std::ranges::views::drop(split, 1)) {
        for (const auto& [k, v] : cur_mesh->m_child_indexer) {
        }
        int64_t index = cur_mesh->m_child_indexer.at(token);
        indices.emplace_back(index);
        cur_mesh = cur_mesh->m_children[index].get();
    }

    return indices;
}

void NamedMultiMesh::set_name(const std::string_view& root_name)
{
    set_names(nlohmann::json(root_name));
}
void NamedMultiMesh::set_names(const nlohmann::json& js)
{
    assert(js.is_object() || js.is_string() || js.is_null());
    m_name_root = std::make_unique<Node>();

    if (js.is_null()) {
        return;
    } else if (js.is_string()) {
        m_name_root->name = js.get<std::string>();
    } else {
        assert(js.is_object());
        assert(js.size() == 1);
        for (const auto& [k, v] : js.items()) {
            m_name_root->name = k;
            m_name_root->set_names(v);
        }
    }
}

NamedMultiMesh::NamedMultiMesh() = default;
NamedMultiMesh::~NamedMultiMesh() = default;
NamedMultiMesh::NamedMultiMesh(NamedMultiMesh&&) = default;
auto NamedMultiMesh::operator=(NamedMultiMesh&&) -> NamedMultiMesh& = default;
} // namespace wmtk::components::input
