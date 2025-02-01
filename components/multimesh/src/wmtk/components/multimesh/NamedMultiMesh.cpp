#include "NamedMultiMesh.hpp"
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <span>
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
    Node& operator=(const Node& o)
    {
        m_children.clear();
        std::ranges::transform(
            o.m_children,
            std::back_inserter(m_children),
            [&](const std::unique_ptr<Node>& n) { return std::make_unique<Node>(*n); });

        update_child_names();
        return *this;
    }
    std::string name;
    std::vector<std::unique_ptr<Node>> m_children;

    std::map<std::string, int64_t> m_child_indexer;
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

    void get_name_tokens(const std::span<const int64_t>& t, std::vector<std::string_view>& toks)
        const
    {
        toks.emplace_back(name);
        if (!t.empty()) {
            const size_t index = t.front();
            assert(index < m_children.size());
            assert(bool(m_children[index]));
            const auto& child = *m_children[index];
            child.get_name_tokens(t.subspan<1>(), toks);
        }
    }


    void update_child_names()
    {
        m_child_indexer.clear();
        for (size_t j = 0; j < m_children.size(); ++j) {
            m_child_indexer.emplace(m_children[j]->name, j);
        }
    }
    friend void to_json(
        nlohmann::json& nlohmann_json_j,
        const NamedMultiMesh::Node& nlohmann_json_t)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto& c_ptr : nlohmann_json_t.m_children) {
            arr.emplace_back(*c_ptr);
        }
        nlohmann_json_j[nlohmann_json_t.name] = arr;
    }
};

NamedMultiMesh::NamedMultiMesh(Mesh& m, const std::string& root_name)
{
    set_mesh(m);
    set_name(root_name);
}
NamedMultiMesh::NamedMultiMesh(Mesh& m, const std::string_view& root_name)
{
    set_mesh(m);
    set_name(root_name);
}
NamedMultiMesh::NamedMultiMesh(Mesh& m, const nlohmann::json& root_name)
{
    set_mesh(m);
    set_names(root_name);
}

void NamedMultiMesh::set_root(Mesh& m)
{
    m_root = m.shared_from_this();
}

void NamedMultiMesh::set_mesh(Mesh& m)
{
    set_root(m);
    // set_root(m.get_multi_mesh_root());
}


Mesh& NamedMultiMesh::get_mesh(const std::string_view& path) const
{
    const auto id = get_id(path);
    return m_root->get_multi_mesh_child_mesh(id);
}
bool NamedMultiMesh::has_mesh(const std::string_view& path) const
{
#if defined(WMTK_ENABLED_CPP20)
    std::ranges::view auto split = internal::split_path(path);
#else
    auto split = internal::split_path(path);
#endif
    Node const* cur_mesh = m_name_root.get();
    assert(*split.begin() == cur_mesh->name || *split.begin() == "");
    for (const auto& token : std::ranges::views::drop(split, 1)) {
        auto it = cur_mesh->m_child_indexer.find(std::string(token));
        if (it == cur_mesh->m_child_indexer.end()) {
            return false;
        } else {
            cur_mesh = cur_mesh->m_children[it->second].get();
        }
    }
    return true;
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
        // try {
        int64_t index = cur_mesh->m_child_indexer.at(std::string(token));
        //} catch(const std::runtime_error& e) {
        //    wmtk::logger().warn("Failed to find mesh named {} in mesh list. Path was ", nmm_name,
        //    path); throw e;
        //}
        indices.emplace_back(index);
        cur_mesh = cur_mesh->m_children[index].get();
    }

    return indices;
}

std::vector<int64_t> NamedMultiMesh::get_id(const Mesh& m) const
{
    return wmtk::multimesh::MultiMeshManager::relative_id(
        m_root->absolute_multi_mesh_id(),

        m.absolute_multi_mesh_id());
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

NamedMultiMesh::NamedMultiMesh()
    : m_name_root(std::make_unique<Node>())
{}
NamedMultiMesh::~NamedMultiMesh() = default;
// NamedMultiMesh::NamedMultiMesh(NamedMultiMesh&&) = default;
// auto NamedMultiMesh::operator=(NamedMultiMesh&&) -> NamedMultiMesh& = default;
auto NamedMultiMesh::operator=(const NamedMultiMesh& o) -> NamedMultiMesh&
{
    m_root = o.m_root;
    m_name_root = std::make_unique<Node>(*o.m_name_root);
    return *this;
}
NamedMultiMesh::NamedMultiMesh(const NamedMultiMesh& o)
    : m_root(o.m_root)
    , m_name_root(std::make_unique<Node>(*o.m_name_root))
{}


std::unique_ptr<nlohmann::json> NamedMultiMesh::get_names_json() const
{
    auto js_ptr = std::make_unique<nlohmann::json>();
    auto& js = *js_ptr;
    js = *m_name_root;


    return js_ptr;
}

std::map<std::string, const Mesh&> NamedMultiMesh::all_meshes() const
// std::map<std::string, std::shared_ptr<const Mesh>> NamedMultiMesh::all_meshes() const
{
    std::map<std::string, const Mesh&> meshes;
    for (const auto& mptr : m_root->get_all_meshes()) {
        meshes.emplace(get_name(*mptr), *mptr);
    }
    return meshes;
}
std::string NamedMultiMesh::get_name(const Mesh& m) const
{
    std::vector<std::string_view> toks;

    const auto id = get_id(m);
    m_name_root->get_name_tokens(id, toks);
    return fmt::format("{}", fmt::join(toks, "."));
}
void NamedMultiMesh::append_child_mesh_names(const Mesh& parent, const NamedMultiMesh& o)
{
    const std::vector<int64_t> parent_id = get_id(parent);
    Node* cur_mesh = m_name_root.get();
    for (const auto& index : parent_id) {
        cur_mesh = cur_mesh->m_children[index].get();
    }

    const auto child_relid = wmtk::multimesh::MultiMeshManager::relative_id(
        parent.absolute_multi_mesh_id(),
        o.root().absolute_multi_mesh_id());
    spdlog::error(
        "{} {} {}",
        fmt::join(parent.absolute_multi_mesh_id(), ","),
        fmt::join(o.root().absolute_multi_mesh_id(), ","),
        fmt::join(child_relid, ","));
    assert(child_relid.size() == 1);

    const int64_t& id = child_relid[0];
    if (const size_t child_size = cur_mesh->m_children.size(); child_size == id) {
        cur_mesh->m_children.emplace_back(std::make_unique<Node>(*o.m_name_root));
    } else if (child_size < id) {
        *cur_mesh->m_children[id] = *o.m_name_root;
    }
    cur_mesh->update_child_names();
}
} // namespace wmtk::components::multimesh
