#include "wmtk/operations/utils/UpdateVertexMultiMeshMapHash.hpp"
#include <wmtk/SimplicialComplex.hpp>

namespace wmtk::operations::utils {
void update_vertex_operation_hashes(Mesh& m, const Tuple& vertex)
{
    // const PrimitiveType pt = m.top_simplex_type();
    // const SimplicialComplex star = SimplicialComplex::closed_star(m, Simplex::vertex(vertex));
    // std::vector<Tuple> tuples_to_update;
    // switch (pt) {
    // case PrimitiveType::Vertex: {
    //     const auto star_vertices = star.get_vertices();
    //     tuples_to_update.reserve(star_vertices.size());
    //     for (const Simplex& s : star_vertices) {
    //         tuples_to_update.emplace_back(s.tuple());
    //     }
    //     break;
    // }
    // case PrimitiveType::Edge: {
    //     break;
    // }
    // }


    // m.update_cell_hashes(tuples_to_update);
    // update_vertex_operation_multimesh_map_hash(m);
}

void update_vertex_operation_multimesh_map_hash(Mesh& m)
{
    // auto& mm_manager = m.m_multi_mesh_manager;
    // const auto parent_hash_accessor = m.get_const_cell_hash_accessor();

    // for (auto& child_data : mm_manager.children()) {
    //     auto& child_mesh = *child_data.mesh;
    //     auto maps = get_map_accessors(m, child_data);
    //     auto& [parent_to_child_accessor, child_to_parent_accessor] = maps;

    //     // resurrect the parent tuples in all mappings
    // }
}

} // namespace wmtk::operations::utils