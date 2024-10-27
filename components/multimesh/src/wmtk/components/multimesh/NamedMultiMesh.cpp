#include "NamedMultiMesh.hpp"
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <wmtk/Mesh.hpp>
#include "internal/split_path.hpp"
namespace wmtk::components::multimesh {

struct NamedMultiMesh::Node
{
    Node(Node&&) = default;
    Node() = default;
    Node(const Node& o)
        : name(o.name)
        , m_child_indexer(o.m_child_indexer)
    {
        std::ranges::transform(
            o.m_children,
            std::back_inserter(m_children),
            [&](const std::unique_ptr<Node>& n) { return std::make_unique<Node>(*n); });
    }
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

#if defined(WMTK_ENABLED_CPP20)
    std::ranges::view auto split = internal::split_path(path);
#else
        auto split = internal::split_path(path);
#endif

    std::vector<int64_t> indices;
    Node const* cur_mesh = m_name_root.get();
    assert(*split.begin() == cur_mesh->name || *split.begin() == "");
    for (const auto& token : std::ranges::views::drop(split, 1)) {
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

std::string_view NamedMultiMesh::root_name() const
{
    assert(bool(m_name_root));

    return m_name_root->name;
}
std::string NamedMultiMesh::name(const std::vector<int64_t>& id) const
{
    std::vector<std::string_view> names;
    Node const* cur_mesh = m_name_root.get();
    names.emplace_back(root_name());
    for (const auto& index : id) {
        cur_mesh = cur_mesh->m_children[index].get();
        names.emplace_back(cur_mesh->name);
    }
    return fmt::format("{}", fmt::join(names, "."));
}

NamedMultiMesh::NamedMultiMesh() = default;
NamedMultiMesh::~NamedMultiMesh() = default;
//NamedMultiMesh::NamedMultiMesh(NamedMultiMesh&&) = default;
//auto NamedMultiMesh::operator=(NamedMultiMesh&&) -> NamedMultiMesh& = default;
auto NamedMultiMesh::operator=(const NamedMultiMesh& o) -> NamedMultiMesh& {
     m_root = o.m_root;
     m_name_root=std::make_unique<Node>(*o.m_name_root);
     return *this;
}
NamedMultiMesh::NamedMultiMesh(const NamedMultiMesh& o)
    : m_root(o.m_root)
    , m_name_root(std::make_unique<Node>(*o.m_name_root))
{}
} // namespace wmtk::components::multimesh
