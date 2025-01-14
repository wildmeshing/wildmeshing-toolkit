#include "Mesh.hpp"
#include <numeric>
#include <queue>

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/vector_hash.hpp>

#include "Primitive.hpp"

namespace wmtk {
std::vector<int64_t> Mesh::absolute_multi_mesh_id() const
{
    return m_multi_mesh_manager.absolute_id();
}
void Mesh::register_child_mesh(
    const std::shared_ptr<Mesh>& child_mesh_ptr,
    const std::vector<std::array<Tuple, 2>>& map_tuples)
{
    m_multi_mesh_manager.register_child_mesh(*this, child_mesh_ptr, map_tuples);
}

void Mesh::deregister_child_mesh(const std::shared_ptr<Mesh>& child_mesh_ptr)
{
    m_multi_mesh_manager.deregister_child_mesh(*this, child_mesh_ptr);
}

void Mesh::update_child_handles()
{
    m_multi_mesh_manager.update_child_handles(*this);
}


bool Mesh::is_from_same_multi_mesh_structure(const Mesh& other) const
{
    return &get_multi_mesh_root() == &other.get_multi_mesh_root();
}

bool Mesh::can_map_up_to(const Mesh& other) const
{
    return is_from_same_multi_mesh_structure(other) &&
           m_multi_mesh_manager.can_map_up(*this, other);
}

bool Mesh::can_map(const Mesh& other_mesh, const simplex::Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.can_map(*this, other_mesh, my_simplex);
}

std::vector<simplex::Simplex> Mesh::map(const Mesh& other_mesh, const simplex::Simplex& my_simplex)
    const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map(*this, other_mesh, my_simplex);
}

std::vector<simplex::Simplex> Mesh::map(
    const Mesh& other_mesh,
    const std::vector<simplex::Simplex>& simplices) const
{
    std::vector<simplex::Simplex> ret;
    ret.reserve(simplices.size());
    for (const auto& s : simplices) {
        auto v = map(other_mesh, s);
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}

std::vector<const Mesh*> Mesh::mappable_meshes(const simplex::Simplex& my_simplex) const
{
    assert(is_valid(my_simplex));
    const Mesh* parent = this;
    while (!parent->is_multi_mesh_root()) {
        parent = parent->m_multi_mesh_manager.m_parent;
    }

    return parent->mappable_child_meshes(map_to_root(my_simplex));
}
std::vector<const Mesh*> Mesh::mappable_child_meshes(const simplex::Simplex& my_simplex) const
{
    assert(is_valid(my_simplex));
    std::vector<const Mesh*> ret;
    for (const auto& child : m_multi_mesh_manager.m_children) {
        const auto& mesh = *child.mesh;
        auto child_simplices = map_to_child(mesh, my_simplex);
        if (child_simplices.empty()) {
            continue;
        }

        for (const auto& s : child_simplices) {
            auto children = mesh.mappable_child_meshes(s);
            ret.insert(ret.end(), children.begin(), children.end());
        }
        ret.emplace_back(&mesh);
    }
    return ret;
}

std::map<const Mesh*, std::vector<simplex::Simplex>> Mesh::map_all(
    const simplex::Simplex& my_simplex) const
{
    assert(is_valid(my_simplex));
    const Mesh* parent = this;
    while (!parent->is_multi_mesh_root()) {
        parent = parent->m_multi_mesh_manager.m_parent;
    }

    return parent->map_all_children(map_to_root(my_simplex));
}
std::map<const Mesh*, std::vector<simplex::Simplex>> Mesh::map_all_children(
    const simplex::Simplex& my_simplex) const
{
    assert(is_valid(my_simplex));
    std::map<const Mesh*, std::vector<simplex::Simplex>> ret;
    for (const auto& child : m_multi_mesh_manager.m_children) {
        const auto& mesh = *child.mesh;
        auto child_simplices = map_to_child(mesh, my_simplex);
        if (child_simplices.empty()) {
            continue;
        }

        for (const auto& s : child_simplices) {
            auto children = mesh.map_all_children(s);
            ret.merge(std::move(children));
        }
        ret[&mesh] = std::move(child_simplices);
    }
    return ret;
}

std::vector<simplex::Simplex> Mesh::lub_map(
    const Mesh& other_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.lub_map(*this, other_mesh, my_simplex);
}

std::vector<simplex::Simplex> Mesh::lub_map(
    const Mesh& other_mesh,
    const std::vector<simplex::Simplex>& simplices) const
{
    std::vector<simplex::Simplex> ret;
    ret.reserve(simplices.size());
    for (const auto& s : simplices) {
        auto v = lub_map(other_mesh, s);
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}

Tuple Mesh::map_up_to_tuples(const Mesh& other_mesh, const Tuple& t) const
{
    return std::get<1>(m_multi_mesh_manager.map_up_to_tuples(*this, other_mesh, t));
}

simplex::Simplex Mesh::map_up_to(const Mesh& other_mesh, const simplex::Simplex& my_simplex) const
{
    return simplex::Simplex(
        my_simplex.primitive_type(),
        map_up_to_tuples(other_mesh, my_simplex.tuple()));
}

simplex::Simplex Mesh::map_to_parent(const simplex::Simplex& my_simplex) const
{
    if (is_multi_mesh_root()) {
        throw std::runtime_error("Attempted to map a simplex to parent despite being a root");
    }
    return m_multi_mesh_manager.map_to_parent(*this, my_simplex);
}
simplex::Simplex Mesh::map_to_root(const simplex::Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root(*this, my_simplex);
}

std::vector<simplex::Simplex> Mesh::map_to_child(
    const Mesh& child_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(child_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    assert(is_valid(my_simplex));
    return m_multi_mesh_manager.map_to_child(*this, child_mesh, my_simplex);
}

std::vector<Tuple> Mesh::map_tuples(const Mesh& other_mesh, const simplex::Simplex& my_simplex)
    const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map_tuples(*this, other_mesh, my_simplex);
}

std::vector<Tuple>
Mesh::map_tuples(const Mesh& other_mesh, PrimitiveType pt, const std::vector<Tuple>& tuples) const
{
    std::vector<Tuple> ret;
    ret.reserve(tuples.size());
    for (const auto& t : tuples) {
        auto v = map_tuples(other_mesh, simplex::Simplex(*this, pt, t));
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}
std::vector<Tuple> Mesh::lub_map_tuples(const Mesh& other_mesh, const simplex::Simplex& my_simplex)
    const
{
    if (!is_from_same_multi_mesh_structure(other_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.lub_map_tuples(*this, other_mesh, my_simplex);
}

std::vector<Tuple> Mesh::lub_map_tuples(
    const Mesh& other_mesh,
    PrimitiveType pt,
    const std::vector<Tuple>& tuples) const
{
    std::vector<Tuple> ret;
    ret.reserve(tuples.size());
    for (const auto& t : tuples) {
        auto v = lub_map_tuples(other_mesh, simplex::Simplex(*this, pt, t));
        ret.insert(ret.end(), v.begin(), v.end());
    }

    return ret;
}

Tuple Mesh::map_to_parent_tuple(const simplex::Simplex& my_simplex) const
{
    if (is_multi_mesh_root()) {
        throw std::runtime_error("Attempted to map a simplex to parent despite being a root");
    }
    return m_multi_mesh_manager.map_to_parent_tuple(*this, my_simplex);
}
Tuple Mesh::map_to_root_tuple(const simplex::Simplex& my_simplex) const
{
    return m_multi_mesh_manager.map_to_root_tuple(*this, my_simplex);
}
std::vector<Tuple> Mesh::map_to_child_tuples(
    const Mesh& child_mesh,
    const simplex::Simplex& my_simplex) const
{
    if (!is_from_same_multi_mesh_structure(child_mesh)) {
        throw std::runtime_error(
            "Attempted to map between two simplices in different multi-mesh structures");
    }
    return m_multi_mesh_manager.map_to_child_tuples(*this, child_mesh, my_simplex);
}

bool Mesh::is_multi_mesh_root() const
{
    return m_multi_mesh_manager.is_root();
}
Mesh& Mesh::get_multi_mesh_root()
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}
const Mesh& Mesh::get_multi_mesh_root() const
{
    return m_multi_mesh_manager.get_root_mesh(*this);
}

Mesh& Mesh::get_multi_mesh_mesh(const std::vector<int64_t>& absolute_id)
{
    return m_multi_mesh_manager.get_mesh(*this, absolute_id);
}
const Mesh& Mesh::get_multi_mesh_mesh(const std::vector<int64_t>& absolute_id) const
{
    return m_multi_mesh_manager.get_mesh(*this, absolute_id);
}


Mesh& Mesh::get_multi_mesh_child_mesh(const std::vector<int64_t>& relative_id)
{
    return m_multi_mesh_manager.get_child_mesh(*this, relative_id);
}
const Mesh& Mesh::get_multi_mesh_child_mesh(const std::vector<int64_t>& relative_id) const
{
    return m_multi_mesh_manager.get_child_mesh(*this, relative_id);
}

std::vector<std::shared_ptr<Mesh>> Mesh::get_child_meshes() const
{
    return m_multi_mesh_manager.get_child_meshes();
}

std::vector<std::shared_ptr<Mesh>> Mesh::get_all_child_meshes() const
{
    std::vector<std::shared_ptr<Mesh>> children;
    auto direct_children = get_child_meshes();
    for (const auto& child : direct_children) {
        children.emplace_back(child);
        auto child_children = child->get_all_child_meshes();
        if (child_children.empty()) {
            continue;
        }
        std::copy(child_children.begin(), child_children.end(), std::back_inserter(children));
    }
    return children;
}

std::vector<std::shared_ptr<const Mesh>> Mesh::get_all_meshes() const
{
    auto meshes2 = get_all_child_meshes();
    std::vector<std::shared_ptr<Mesh const>> meshes;
    meshes.emplace_back(shared_from_this());
    for (const auto& m : meshes2) {
        meshes.emplace_back(m);
    }
    return meshes;
    // std::queue<std::shared_ptr<Mesh const>> queue;
    ////std::queue<Mesh const*> queue;
    // meshes.emplace_back(this);
    // while(!queue.empty()) {
    //     const auto& cur = queue.front();
    //    //Mesh const* cur = queue.front();
    //    queue.pop();
    //    meshes.emplace_back(cur->shared_from_this());
    //    for(const auto& m: cur->get_child_meshes()) {
    //        queue.emplace(m.get());
    //    }
    //}
    // return meshes;
}
} // namespace wmtk
