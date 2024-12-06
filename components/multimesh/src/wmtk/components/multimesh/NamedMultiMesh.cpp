#include "NamedMultiMesh.hpp"
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <span>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>
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
            const auto& n = m_children[j]->name;
            if (m_child_indexer.contains(n)) {
                throw std::runtime_error(fmt::format("Child indexer saw the name {} twice", name));
            }
            m_child_indexer.emplace(n, j);
        }
    }


    std::vector<std::string> get_all_paths(const std::string_view& prefix = "") const
    {
        std::vector<std::string> paths;

        std::string path;
        if (prefix.empty()) {
            path = this->name;
        } else {
            path = fmt::format("{}.{}", prefix, this->name);
        }
        paths.emplace_back(path);
        for (const auto& c : m_children) {
            auto n2 = c->get_all_paths(path);
            std::copy(n2.begin(), n2.end(), std::back_inserter(paths));
        }

        return paths;
    }

    friend void to_json(
        nlohmann::json& nlohmann_json_j,
        const NamedMultiMesh::Node& nlohmann_json_t)
    {
        nlohmann::json value;
        for (const auto& c_ptr : nlohmann_json_t.m_children) {
            value.update(*c_ptr);
        }
        value["ptr"] = fmt::format("{}", fmt::ptr(&nlohmann_json_t));
        nlohmann_json_j[nlohmann_json_t.name] = value;
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
    populate_default_names();
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
    const std::string& cur_name = cur_mesh->name;
    const bool same_name = *split.begin() == cur_mesh->name;
    const bool empty_name = *split.begin() == "";
    if (!(same_name || empty_name)) {
        return false;
    }
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
    const bool same_name = *split.begin() == cur_mesh->name;
    const bool empty_name = *split.begin() == "";
    if (!(same_name || empty_name)) {
        throw std::out_of_range(fmt::format(
            "Root name [{}] of path [{}] was not either empty or [{}]",
            *split.begin(),
            path,
            cur_mesh->name));
    }
    for (const auto& token : std::ranges::views::drop(split, 1)) {
        try {
            if (cur_mesh->m_child_indexer.size() == 0) {
                if (!cur_mesh->m_children.empty()) {
                    throw std::runtime_error(fmt::format(
                        "Child indexer wasn't initialized after children were populated (child "
                        "indexer size {} and child size {} different)",
                        cur_mesh->m_child_indexer.size(),
                        cur_mesh->m_children.size()));
                } else {
                    throw std::out_of_range(fmt::format(
                        "Could not parse {} from name {} because child indexer was empty\nFull "
                        "Tree: {}\nCurrent subtree: {}",
                        token,
                        path,
                        nlohmann::json(*m_name_root).dump(2),
                        nlohmann::json(*cur_mesh).dump(2)));
                }
            }

            int64_t index = cur_mesh->m_child_indexer.at(std::string(token));
            indices.emplace_back(index);
            cur_mesh = cur_mesh->m_children[index].get();
        } catch (const std::out_of_range& e) {
            wmtk::logger().warn(
                "Failed to find mesh named {} in mesh list. Path was {}",
                token,
                path);
            throw e;
        }
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

void NamedMultiMesh::populate_default_names()
{
    if (!bool(m_name_root)) {
        m_name_root = std::make_unique<Node>();
    }
    std::function<void(Node&, const Mesh& m)> sdn;
    sdn = [&sdn](Node& n, const Mesh& m) {
        const auto& children = m.get_child_meshes();
        if (n.m_children.size() < children.size()) {
            n.m_children.resize(children.size());
        }

        for (size_t j = 0; j < children.size(); ++j) {
            auto& nnptr = n.m_children[j];
            if (!bool(nnptr)) {
                nnptr = std::make_unique<Node>();
            }
            auto& nn = *nnptr;
            if (nn.name.empty()) {
                nn.name = fmt::format("{}", j);
            }
        }
        for (size_t j = 0; j < children.size(); ++j) {
            sdn(*n.m_children[j], *children[j]);
        }
        n.update_child_names();
    };
    assert(bool(m_name_root));
    if (m_name_root->name.empty()) {
        m_name_root->name = "0";
    }
    sdn(*m_name_root, *m_root);
}

std::string_view NamedMultiMesh::root_name() const
{
    assert(bool(m_name_root));

    return m_name_root->name;
}
std::string NamedMultiMesh::name(const std::vector<int64_t>& id) const
{
    return get_name(id);
}
std::string NamedMultiMesh::get_name(const std::vector<int64_t>& id) const
{
    std::vector<std::string_view> names;
    Node const* cur_mesh = m_name_root.get();
    names.emplace_back(root_name());
    for (const auto& index : id) {
        cur_mesh = cur_mesh->m_children.at(index).get();
        names.emplace_back(cur_mesh->name);
    }
    return fmt::format("{}", fmt::join(names, "."));
}

auto NamedMultiMesh::get_node(const std::vector<int64_t>& id) const -> const Node&
{
    Node const* cur_mesh = m_name_root.get();
    for (const auto& index : id) {
        cur_mesh = cur_mesh->m_children[index].get();
    }
    return *cur_mesh;
}
auto NamedMultiMesh::get_node(const std::vector<int64_t>& id) -> Node&
{
    Node* cur_mesh = m_name_root.get();
    for (const auto& index : id) {
        cur_mesh = cur_mesh->m_children[index].get();
    }
    return *cur_mesh;
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


std::unique_ptr<nlohmann::json> NamedMultiMesh::get_names_json(const std::string_view& path) const
{
    auto js_ptr = std::make_unique<nlohmann::json>();
    auto& js = *js_ptr;
    const auto id = get_id(path);
    js = get_node(id);


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
    assert(child_relid.size() == 1);

    const int64_t& id = child_relid[0];
    if (const size_t child_size = cur_mesh->m_children.size(); child_size == id) {
        cur_mesh->m_children.emplace_back(std::make_unique<Node>(*o.m_name_root));
    } else if (child_size < id) {
        *cur_mesh->m_children[id] = *o.m_name_root;
    }
    cur_mesh->update_child_names();
}
bool NamedMultiMesh::is_valid(bool pass_exceptions) const
{
    // check that every mesh has a valid name
    auto throw_or_except = [pass_exceptions](const auto& e) {
        if (pass_exceptions) {
            throw e;
        }
        return false;
    };
    for (const auto& mptr : m_root->get_all_meshes()) {
        try {
            wmtk::logger().trace(
                "checking if NamedMultiMesh is valid for mesh with relative path [{}] and local "
                "path [{}]",
                fmt::join(get_id(*mptr), ","),
                fmt::join(mptr->absolute_multi_mesh_id(), ","));
            get_name(*mptr);
        } catch (const std::range_error& e) {
            return throw_or_except(e);
        } catch (const std::runtime_error& e) {
            return throw_or_except(e);
        }
    }

    const auto all_paths = m_name_root->get_all_paths();
    if (all_paths.empty()) {
        nlohmann::json js = *m_name_root;
        throw std::runtime_error("No paths exist in mesh");
    }
    for (const std::string& path : all_paths) {
        try {
            wmtk::logger().trace("checking if NamedMultiMesh is valid for path {}", path);
            get_mesh(path);
        } catch (const std::runtime_error& e) {
            return throw_or_except(e);
        }
    }

    return true;
}
} // namespace wmtk::components::multimesh
