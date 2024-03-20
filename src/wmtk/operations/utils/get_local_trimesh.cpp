// #include "get_local_trimesh.hpp"
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/TriMesh.hpp>
#include <unordered_map>

// TODO: we also need fid_maps
namespace wmtk::operations::utils {
    std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t> >
    get_local_trimesh(const wmtk::TriMesh& mesh, const wmtk::simplex::Simplex& simplex)
    {
        auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto pos = mesh.create_const_accessor<double>(pos_handle);

        std::unordered_map<int64_t, int> global_to_local_map;    
        const auto cofaces = wmtk::simplex::top_dimension_cofaces(mesh, simplex).simplex_vector(PrimitiveType::Triangle);
        
        
        Eigen::MatrixXi F(cofaces.size(), 3);
        int vertex_count = 0;
        int face_count = 0;
        for (const auto& f_tuple : cofaces)
        {
            // get 3 vertices
            Tuple cur_v = f_tuple.tuple();
            for (int i = 0; i < 3; i++)
            {    
                int64_t global_vid = mesh.id(wmtk::simplex::Simplex::vertex(cur_v));            
                if (global_to_local_map.count(global_vid) == 0)
                {
                    global_to_local_map[global_vid] = vertex_count;
                    vertex_count++;
                }
                F(face_count, i) = global_to_local_map[global_vid];
                cur_v = mesh.switch_tuples(cur_v, {PrimitiveType::Edge, PrimitiveType::Vertex}); // next vertex
            }
            face_count++;
        }

        Eigen::MatrixXd V(vertex_count, 3);
        std::vector<int64_t> local_to_global(vertex_count);

        // build V, local_to_global
        for (const auto& pair : global_to_local_map)
        {
            local_to_global[pair.second] = pair.first;
            V.row(pair.second) = pos.const_vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, pair.first));
        }

        return std::make_tuple(F, V, local_to_global);
    }
} // namespace wmtk::operations::utils