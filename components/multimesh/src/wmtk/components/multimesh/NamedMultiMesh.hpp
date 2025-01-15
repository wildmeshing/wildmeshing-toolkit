#pragma once
#include <memory>
#include <nlohmann/json_fwd.hpp>

namespace wmtk {
class Mesh;
namespace attribute {
class MeshAttributeHandle;
}
} // namespace wmtk


namespace wmtk::components::multimesh {

// allows for accessing a mesh by a string ID
// The IDs are loaded through a json format, and hte root node can either have a name or be hte
// empty string. Even if hte root it has a name, typing it can be skipped by a preceding '.' before
// a path
// By default each mesh is named by its index number
class NamedMultiMesh
{
public:
    NamedMultiMesh();
    NamedMultiMesh(Mesh& m, const std::string_view& root_name, bool do_populate_child_names = true);
    explicit NamedMultiMesh(
        Mesh& m,
        const nlohmann::json& root_name,
        bool do_populate_missing_names = true);

    // Explicit constructors to remove ambiguities between string_view and json constructors
    explicit NamedMultiMesh(
        Mesh& m,
        const std::string& root_name,
        bool do_populate_child_names = true);
    template <size_t N>
    explicit NamedMultiMesh(Mesh& m, const char name[N])
        : NamedMultiMesh(m, std::string_view(name))
    {}
    // NamedMultiMesh(NamedMultiMesh&&);
    NamedMultiMesh(const NamedMultiMesh&);
    ~NamedMultiMesh();
    // auto operator=(NamedMultiMesh&&) -> NamedMultiMesh&;
    auto operator=(const NamedMultiMesh&) -> NamedMultiMesh&;

    /// sets just the name of the root mesh, keeping child names the same
    void set_name(const std::string_view& root_name = "");
    void set_names(const nlohmann::json& js);
    void populate_default_names();
    void set_root(Mesh& m, bool do_populate_unnamed = true);
    void append_child_mesh_names(const Mesh& parent, const NamedMultiMesh& o);

    std::unique_ptr<nlohmann::json> get_names_json(const std::string_view& path = "") const;

    std::string_view root_name() const;
    std::string name(const std::vector<int64_t>& id) const;
    std::string get_name(const std::vector<int64_t>& id) const;
    bool has_name(const std::vector<int64_t>& id) const;

    /// Navigates to the root of the multimesh
    void set_mesh(Mesh& m, bool do_populate_unnamed = true);
    Mesh& get_mesh(const std::string_view& path) const;
    bool has_mesh(const std::string_view& path) const;
    std::vector<int64_t> get_id(const std::string_view& path) const;
    std::vector<int64_t> get_id(const Mesh& m) const;

    Mesh& root() { return *m_root; }
    const Mesh& root() const { return *m_root; }


    std::map<std::string, const Mesh&> all_meshes() const;
    std::map<std::string, Mesh&> all_meshes();

    // returns the name of a mesh if it lies in this multimesh
    std::string get_name(const Mesh& m) const;

    bool has_name(const Mesh& m) const;

    std::string get_path(const wmtk::attribute::MeshAttributeHandle& m) const;

    // checks whether the meshes / nodes are synchronized. Passes thrown errors if desired
    bool is_valid(bool pass_exceptions = false) const;


private:
    struct Node;
    const Node& get_node(const std::vector<int64_t>& id) const;
    Node& get_node(const std::vector<int64_t>& id);

private:
    std::shared_ptr<Mesh> m_root;
    std::unique_ptr<Node> m_name_root;
};

} // namespace wmtk::components::multimesh
