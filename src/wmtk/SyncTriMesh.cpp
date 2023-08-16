#include "SyncTriMesh.hpp"
#include "SimplicialComplex.hpp"

namespace wmtk {

    void SubTriMesh::init_map_to_main_mesh(Eigen::Ref<const VectorXl> face_map, const TriMesh& main_mesh)
    {
        auto map_to_main_mesh_handle = register_attribute<long>("map_to_main_mesh", PrimitiveType::Face, 5);
        map_to_main_mesh_accessor = create_accessor(map_to_main_mesh_handle);

        for (long i = 0; i < face_map.size(); ++i)
        {
            auto main_mesh_face_tuple = main_mesh.face_tuple_from_id(face_map[i]);
            auto map_to_main = map_to_main_mesh_accessor.vector_attribute(i)
            map_to_main(0) = main_mesh_face_tuple.local_vid;
            map_to_main(1) = main_mesh_face_tuple.local_eid;
            map_to_main(2) = main_mesh_face_tuple.local_fid;
            map_to_main(3) = main_mesh_face_tuple.global_cid;
            map_to_main(4) = main_mesh_face_tuple.hash;
        }
    }

    void SyncTriMesh::initizalize(Eigen::Ref<const RowVectors3l> F, std::vector<Eigen::Ref<const RowVectors3l>> sub_Fs, std::vector<Eigen::Ref<const VectorXl> face_maps)
    {
        
        TriMesh::initialize(F);

        for (int i = 0; i < sub_Fs.size(); ++i)
        {
            auto face_map_to_main = face_maps[i];
            TriMesh sub_trimesh;
            sub_trimesh.initialize(sub_Fs[i]);
            sub_trimesh.init_map_to_main_mesh(face_map_to_main, *this);

            auto map_to_sub_trimesh_handle = register_attribute<long>("map_to_sub_mesh"+std::to_string(i), PrimitiveType::Face, 5);
            map_to_sub_trimesh_accessor = create_accessor(map_to_sub_trimesh_handle);

            for (long sub_mesh_fid = 0; sub_mesh_fid < face_maps[i].size(); ++sub_mesh_fid)
            {
                auto sub_mesh_face_tuple = sub_trimesh.face_tuple_from_id(sub_mesh_fid);
                long main_mesh_fid = face_map_to_main[sub_mesh_fid];

                auto map_to_sub_trimesh = map_to_sub_trimesh_mesh_accessor.vector_attribute(main_mesh_fid);
                map_to_sub_trimesh(0) = sub_mesh_face_tuple.local_vid;
                map_to_sub_trimesh(1) = sub_mesh_face_tuple.local_eid;
                map_to_sub_trimesh(2) = sub_mesh_face_tuple.local_fid;
                map_to_sub_trimesh(3) = sub_mesh_face_tuple.global_cid;
                map_to_sub_trimesh(4) = sub_mesh_face_tuple.hash;
            }

            sub_trimeshes.push_back(sub_trimesh);
            maps_to_sub_trimeshes_accessors.push_back(map_to_main_mesh_accessor);
        }

    }

    Tuple map_tuple(const Tuple& source_tuple, const Tuple& cannoic_tuple_in_source_mesh, const Tuple& cannoic_tuple_in_target_mesh, const TriMesh& source_mesh, const TriMesh target_mesh)
    {
        Tuple ret_tuple = cannoic_tuple_in_target_mesh;
        Tuple cur_tuple = source_tuple;
        std::vector<char> record_switch_operations;
        while (cur_tuple != cannoic_tuple_in_source_mesh)
        {
            cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Vertex);
            record_switch_operations.push_back('v');
            if (cur_tuple == cannoic_tuple_in_source_mesh)
            {
                break;
            }

            cur_tuple = source_mesh.switch_tuple(cur_tuple, PrimitiveType::Edge);
            record_switch_operations.push_back('e');
        }

        for (int i = record_switch_operations.size()-1; i >= 0; --i)
        {
            if (record_switch_operations[i] == 'v')
            {
                ret_tuple = target_mesh.switch_tuple(ret_tuple, PrimitiveType::Vertex);
            }
            else
            {
                ret_tuple = target_mesh.switch_tuple(ret_tuple, PrimitiveType::Edge);
            }
        }

        return ret_tuple;
    }

    Tuple SyncTriMesh::map_tuple_from_main_mesh_to_sub_mesh(const Tuple& t, int sub_mesh_id) const
    {
        auto submesh = sub_trimeshes[sub_mesh_id];
        Tuple main_mesh_cannonic_tuple = Tuple(sub_trimeshes.map_to_main_mesh_accessor.vector_attribute(t));
        Tuple sub_mesh_cannoic_tuple = sub_trimeshes.face_tuple_from_id(t.global_cid);
        
        return map_tuple(t, main_mesh_cannonic_tuple, sub_mesh_cannoic_tuple, submesh, *this);
    }

    Tuple SyncTriMesh::map_tuple_from_sub_meshes_to_main_mesh(const Tuple& t, int sub_mesh_id) const
    {
        auto submesh = sub_trimeshes[sub_mesh_id];
        Tuple main_mesh_cannonic_tuple = face_tuple_from_id(t.global_cid);
        Tuple sub_mesh_cannoic_tuple = Tuple(maps_to_sub_trimesh_accesors[sub_mesh_id].vector_attribute(t));

        return map_tuple(t, sub_mesh_cannoic_tuple, main_mesh_cannonic_tuple, *this, submesh);
    }

 /*
    std::vector<long> get_new_fids_after_split(const Tuple& ret_tuple) const
    {
        std::vector<long> new_fids;
        new_fids.push_back(ret_tuple.global_cid);
        new_fids.push_back(switch_face(switch_edge(switch_vertex(ret_tuple))).global_cid);
        if (!is_boundary(t))
        {
            new_fids.push_back(switch_face(ret_tuple).global_cid);
            new_fids.push_back(switch_face(switch_edge(switch_vertex(switch_face(ret_tuple)))).global_cid);
        }
        return new_fids;
    }
*/
    Tuple SyncTriMesh::split_edge(const Tuple& t)
    {
        // split t in main_mesh
        TriMesh::TriMeshOperationExecutor executor_main_mesh(*this, t);
        Tuple ret_tuple_main_mesh = executor_main_mesh.split_edge();
        auto new_fids_main_mesh = get_new_fids_after_split(ret_tuple_main_mesh);
        
        for (size_t submesh_id = 0; submesh_id < sub_trimeshes.size(); ++submesh_id)
        {
            auto submesh = sub_trimeshes[submesh_id];
            const Tuple submesh_t = map_tuple_from_main_mesh_to_sub_mesh(t, submesh_id);
            std::vector<long> new_fids_sub_mesh;

            if (!submesh.is_boundary(submesh_t))
            {
                // split t in submesh
                TriMesh::TriMeshOperationExecutor executor_sub_mesh(submesh, submesh_t);
                auto ret_tuple_sub_mesh = executor_sub_mesh.split_edge();
                new_fids_sub_mesh = get_new_fids_after_split(ret_tuple_sub_mesh);
            }
            else if (!is_boundary(t))
            {
                const Tuple main_mesh_t_opp = switch_tuple(t, PrimitiveType::Face);
                const Tuple submesh_t_opp = map_tuple_from_main_mesh_to_sub_mesh(main_mesh_t_opp, submesh_id);

                // split t in submesh
                TriMesh::TriMeshOperationExecutor executor_sub_mesh(submesh, submesh_t);
                auto ret_tuple_sub_mesh = executor_sub_mesh.split_edge();
                new_fids_sub_mesh = get_new_fids_after_split(ret_tuple_sub_mesh);

                // split t_opp in submesh
                TriMesh::TriMeshOperationExecutor executor_sub_mesh_opp(submesh, submesh_t_opp);
                auto ret_tuple_sub_mesh_opp = executor_sub_mesh.split_edge();
                auto new_fids_sub_mesh_opp = get_new_fids_after_split(ret_tuple_sub_mesh_opp);

                // get full list of new fids
                new_fids_sub_mesh.insert(new_fids_sub_mesh.end(), new_fids_sub_mesh_opp.begin(), new_fids_sub_mesh_opp.end());
            }

            // update maps
            for (size_t i = 0; i < new_fids_sub_mesh.size(); ++i)
            {
                auto sub_mesh_fid = new_fids_sub_mesh[i];
                auto main_mesh_fid = new_fids_main_mesh[i];
                Tuple main_mesh_face_tuple = face_tuple_from_id(main_mesh_fid);
                Tuple sub_mesh_face_tuple = submesh.face_tuple_from_id(sub_mesh_fid);

                auto map_to_main = submesh.map_to_main_mesh_accessor.vector_attribute(sub_mesh_fid);
                map_to_main(0) = main_mesh_face_tuple.local_vid;
                map_to_main(1) = main_mesh_face_tuple.local_eid;
                map_to_main(2) = main_mesh_face_tuple.local_fid;
                map_to_main(3) = main_mesh_face_tuple.global_cid;
                map_to_main(4) = main_mesh_face_tuple.hash;

                auto map_to_sub_trimesh = maps_to_sub_trimesh_accessors[submesh_id].vector_attribute(main_mesh_fid);
                map_to_sub_trimesh(0) = sub_mesh_face_tuple.local_vid;
                map_to_sub_trimesh(1) = sub_mesh_face_tuple.local_eid;
                map_to_sub_trimesh(2) = sub_mesh_face_tuple.local_fid;
                map_to_sub_trimesh(3) = sub_mesh_face_tuple.global_cid;
                map_to_sub_trimesh(4) = sub_mesh_face_tuple.hash;
            }

        }

        // TODO: update all tuple's hashes in map_to_main_mesh and maps_to_sub_trimesh
        udpate_all_tuple_hashes();

        return ret_tuple_main_mesh;
    }

    Tuple SyncTriMesh::collapse_edge(const Tuple& t)
    {
        //TODO: in before(), need to check link_conditions for all sub_meshes

        TriMesh::TriMeshOperationExecutor executor_main_mesh(*this, t);
        Tuple ret_tuple_main_mesh = executor_main_mesh.collapse_edge();

        for (size_t submesh_id = 0; submesh_id < sub_trimeshes.size(); ++submesh_id)
        {
            auto submesh = sub_trimeshes[submesh_id];
            const Tuple submesh_t = map_tuple_from_main_mesh_to_sub_mesh(t, submesh_id);

            if (!submesh.is_boundary(submesh_t))
            {
                TriMesh::TriMeshOperationExecutor executor_sub_mesh(submesh, submesh_t);
                executor_sub_mesh.collapse_edge();
            }
            else if (!is_boundary(t))
            {
                const Tuple main_mesh_t_opp = switch_tuple(t, PrimitiveType::Face);
                const Tuple submesh_t_opp = map_tuple_from_main_mesh_to_sub_mesh(main_mesh_t_opp, submesh_id);
                TriMesh::TriMeshOperationExecutor executor_sub_mesh(submesh, submesh_t);
                executor_sub_mesh.collapse_edge();
                TriMesh::TriMeshOperationExecutor executor_sub_mesh_opp(submesh, submesh_t_opp);
                executor_sub_mesh.collapse_edge();
            }
        }
        // TODO: update all tuple's hashes in map_to_main_mesh and maps_to_sub_trimesh
        udpate_all_tuple_hashes();
        return ret_tuple_main_mesh;
    }

    bool SyncTriMesh::is_sub_meshes_valid() const
    {
        for (size_t submesh_id = 0; submesh_id < sub_trimeshes.size(); ++submesh_id)
        {
            auto sub_mesh = sub_trimeshes[submesh_id];

            auto check_tuple = [&](Tuple _t){
                auto main_t = map_tuple_from_sub_meshes_to_main_mesh(_t, submesh_id);
                if (is_boundary(main_t)) 
                {
                    if (!sub_mesh.is_boundary(_t))
                    return false;
                }
                else
                {
                    if (sub_mesh.is_boundary(_t))
                    return true;
                    else
                    {
                        auto main_t_opp = switch_tuple(main_t, PrimitiveType::Face);
                        auto _t_opp = sub_mesh.switch_tuple(_t, PrimitiveType::Face);

                        return main_t_opp.same_ids(map_tuple_from_sub_meshes_to_main_mesh(_t_opp, submesh_id));
                    }
                }
                return true;
            };

            for (long i = 0; i < sub_mesh.capacity(PrimitiveType::Face); ++i)
            {
                auto t = sub_mesh.face_tuple_from_id(i);
                auto tn = sub_mesh.switch_tuple(sub_mesh.switch_tuple(t, PrimitiveType::Vertex), PrimitiveType::Edge);
                auto tnn = sub_mesh.switch_tuple(t, PrimitiveType::Edge);
                
                if (!check_tuple(t) || !check_tuple(tn) || !check_tuple(tnn))
                {
                    return false;
                }
            }
        }
        return true;
    }

}// namespace wmtk