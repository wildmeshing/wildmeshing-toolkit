#pragma once
#include <memory>
#include <nlohmann/json_fwd.hpp>

namespace wmtk {
class Mesh;
}


namespace wmtk::components::input {

class NamedMultiMesh
{
    NamedMultiMesh();
    ~NamedMultiMesh();

    void set_names(const nlohmann::json& js, const std::string_view& name = "");

    Mesh& root() { return *m_root; }
    const Mesh& root() const { return *m_root; }

private:
    struct Node;
    std::shared_ptr<Mesh> m_root;
    std::unique_ptr<Node> m_name_root;
};

} // namespace wmtk::components::input
