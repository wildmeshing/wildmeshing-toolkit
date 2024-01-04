#include "extract_child_mesh_from_tag.hpp"
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/MultiMeshManager.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/attribute/MeshAttributes.hpp>
#include "internal/TupleTag.hpp"

namespace wmtk::multimesh::utils {

std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag(
    Mesh& m,
    const std::string& tag,
    const int64_t tag_value,
    const PrimitiveType pt)
{
    assert(m.top_simplex_type() >= pt);
    auto tag_handle = m.get_attribute_handle<int64_t>(tag, pt).as<int64_t>();
    return extract_and_register_child_mesh_from_tag_handle(m, tag_handle, tag_value);
}


std::shared_ptr<Mesh> internal::TupleTag::extract_and_register_child_mesh_from_tag_handle(
    Mesh& m,
    const wmtk::attribute::TypedAttributeHandle<int64_t>& tag_handle,
    const int64_t tag_value)
{
    auto tags = m.create_const_accessor(tag_handle);
    const PrimitiveType pt = tag_handle.primitive_type();

    std::vector<Tuple> tagged_tuples;
    for (const Tuple& t : m.get_all(pt)) {
        if (tags.const_scalar_attribute(t) == tag_value) {
            tagged_tuples.emplace_back(t);
        }
    }

    switch (pt) {
    case PrimitiveType::Vertex: throw("not implemented");
    case PrimitiveType::Edge: {
        std::map<int64_t, int64_t> parent_to_child_vertex_map;
        // int64_t child_vertex_count = 0;

        RowVectors2l edge_mesh_matrix;
        edge_mesh_matrix.resize(tagged_tuples.size(), 2);

        for (size_t i = 0; i < tagged_tuples.size(); ++i) {
            const std::array<int64_t, 2> vs = {
                {m.id(tagged_tuples[i], PrimitiveType::Vertex),
                 m.id(m.switch_vertex(tagged_tuples[i]), PrimitiveType::Vertex)}};


            // check and add v0, v1 to the vertex map
            for (int k = 0; k < 2; k++) {
                size_t size = parent_to_child_vertex_map.size();
                parent_to_child_vertex_map.try_emplace(vs[k], size);
                edge_mesh_matrix(i, k) = parent_to_child_vertex_map[vs[k]];
            }
        }

        std::shared_ptr<EdgeMesh> child_ptr = std::make_shared<EdgeMesh>();
        auto& child = *child_ptr;
        child.initialize(edge_mesh_matrix);

        std::vector<std::array<Tuple, 2>> child_to_parent_map(tagged_tuples.size());
        assert(tagged_tuples.size() == child.capacity(PrimitiveType::Edge));

        const auto edgemesh_edge_tuples = child.get_all(PrimitiveType::Edge);

        for (size_t i = 0; i < tagged_tuples.size(); ++i) {
            child_to_parent_map[i] = {{edgemesh_edge_tuples[i], tagged_tuples[i]}};
        }

        m.register_child_mesh(child_ptr, child_to_parent_map);
        return child_ptr;
    }
    case PrimitiveType::Face: {
        std::map<int64_t, int64_t> parent_to_child_vertex_map;
        int64_t child_vertex_count = 0;

        RowVectors3l tri_mesh_matrix;
        tri_mesh_matrix.resize(tagged_tuples.size(), 3);
        for (int64_t i = 0; i < tagged_tuples.size(); ++i) {
            // TODO: check if this break the orientation of the map
            const std::array<int64_t, 3> vs = {
                {m.id(tagged_tuples[i], PrimitiveType::Vertex),
                 m.id(m.switch_vertex(tagged_tuples[i]), PrimitiveType::Vertex),
                 m.id(m.switch_vertex(m.switch_edge(tagged_tuples[i])), PrimitiveType::Vertex)}};

            // check and add v0, v1, v2 to the vertex map
            for (int k = 0; k < 3; k++) {
                size_t size = parent_to_child_vertex_map.size();
                parent_to_child_vertex_map.try_emplace(vs[k], size);
                tri_mesh_matrix(i, k) = parent_to_child_vertex_map[vs[k]];
            }
        }
        std::shared_ptr<TriMesh> child_ptr = std::make_shared<TriMesh>();
        auto& child = *child_ptr;
        child.initialize(tri_mesh_matrix);
        std::vector<std::array<Tuple, 2>> child_to_parent_map(tagged_tuples.size());
        assert(tagged_tuples.size() == child.capacity(PrimitiveType::Face));

        auto trimesh_face_tuples = child.get_all(PrimitiveType::Face);

        for (int64_t i = 0; i < tagged_tuples.size(); ++i) {
            child_to_parent_map[i] = {{trimesh_face_tuples[i], tagged_tuples[i]}};
        }

        m.register_child_mesh(child_ptr, child_to_parent_map);
        return child_ptr;
    }
    case PrimitiveType::Tetrahedron: throw("not implemented");
    case PrimitiveType::HalfEdge:
    default: throw("invalid child mesh type");
    }
}


std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag_handle(
    Mesh& m,
    const wmtk::attribute::TypedAttributeHandle<int64_t>& tag_handle,
    const int64_t tag_value)
{
    return internal::TupleTag::extract_and_register_child_mesh_from_tag_handle(m,tag_handle, tag_value);
}

} // namespace wmtk::multimesh::utils
