#pragma once
#include "TriMesh.hpp"


namespace wmtk {
    class SubTriMesh : public TriMesh
    {
        public:
        // per face attribute map to main mesh
        // for face i, map_to_main_mesh(face_tuple_from_id(i)) == main_mesh.face_tuple_from_id(face_map(i))

        void init_map_to_main_mesh(Eigen::Ref<const VectorXl> face_map, const TriMesh& main_mesh);
        
        Accessor<long> map_to_main_mesh_accessor;
    }

    class SyncTriMesh : public TriMesh
    {
        public:
        std::vector<SubTriMesh> sub_trimeshes;


        // TODO: current assume cannonical_tuple[i] = face_tuple_from_id(i)
        // Can also store it as an attribute explicitly
        // In current implementation, the map can be easily store as just a face_id map (not sure need discussion)



        // per face attribute maps to sub_meshes
        // for face i, map_to_sub_meshes(face_tuple_from_id(i)) == sub_mesh.face_tuple_from_id(face_map(i))
        std::vector<Accessor<long>> maps_to_sub_trimesh_accessors;
        
        Tuple map_tuple_from_main_mesh_to_sub_meshes(const Tuple& t, int sub_mesh_id) const;
        Tuple map_tuple_from_sub_meshes_to_main_mesh(const Tuple& t, int sub_mesh_id) const;

        Tuple map_tuple_between_sub_meshes(const Tuple& t, int sub_mesh_id_source, int sub_mesh_id_target) const
        {
            return map_tuple_from_main_mesh_to_sub_meshes(map_tuple_from_sub_meshes_to_main_mesh(t, sub_mesh_id_source), sub_mesh_id_target);
        };

        void initizalize(Eigen::Ref<const RowVectors3l> F, std::vector<Eigen::Ref<const RowVectors3l>> sub_Fs, std::vector<Eigen::Ref<const VectorXl>> face_maps);


        Tuple split_edge(const Tuple& t) override;
        Tuple collapse_edge(const Tuple& t) override;

        // check if sub_mesh is a valid sub_mesh of this mesh
        bool is_sub_meshes_valid() const;
        
    };
}// namespace wmtk