#pragma once
#include <memory>
#include <nlohmann/json_fwd.hpp>

namespace wmtk {
class Mesh;
}


namespace wmtk::components::multimesh {

// allows for accessing a mesh by a string ID
// The IDs are loaded through a json format, and hte root node can either have a name or be hte
// empty string. Even if hte root it has a name, typing it can be skipped by a preceding '.' before
// a path
class NamedMultiMesh
{
public:
    NamedMultiMesh();
    //NamedMultiMesh(NamedMultiMesh&&);
    NamedMultiMesh(const NamedMultiMesh&);
    ~NamedMultiMesh();
   // auto operator=(NamedMultiMesh&&) -> NamedMultiMesh&;
   auto operator=(const NamedMultiMesh&) -> NamedMultiMesh&;

    void set_name(const std::string_view& root_name = "");
    void set_names(const nlohmann::json& js);
    void set_root(Mesh& m);

    std::string_view root_name() const;
    std::string name(const std::vector<int64_t>& id) const;

    /// Navigates to the root of the multimesh
    void set_mesh(Mesh& m);
    Mesh& get_mesh(const std::string_view& path) const;
    std::vector<int64_t> get_id(const std::string_view& path) const;

    Mesh& root() { return *m_root; }
    const Mesh& root() const { return *m_root; }

private:
    struct Node;
    std::shared_ptr<Mesh> m_root;
    std::unique_ptr<Node> m_name_root;
};

} // namespace wmtk::components::multimesh
