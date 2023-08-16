#include "SyncTriMesh.hpp"
#include "SimplicialComplex.hpp"

namespace wmtk {
    void SyncTriMesh::initizalize(Eigen::Ref<const RowVectors3l> F, std::vector<Eigen::Ref<const RowVectors3l>> F_cuts)
    {
        
        TriMesh::initialize(F);
        for (int i = 0; i < F_cuts.size(); ++i)
        {

            if (F.rows() != F_cuts[i].rows())
            {
                std::cout << "error: F and F_cut must have the same number of rows" << std::endl;
                return;
            }

            TriMesh cut_mesh;
            cut_mesh.initialize(F_cuts[i]);
            cut_meshes.push_back(cut_mesh);
        }
        if (!is_cut_meshes_valid())
        {
            std::cout << "error: cut meshes not valid" << std::endl;
        }
    }

    bool SyncTriMesh::is_cut_meshes_valid() const
    {
        for (auto cut_mesh : cut_meshes)
        {

            auto check_tuple = [&](Tuple _t){
                if (is_boundary(_t)) 
                {
                    if (!cut_mesh.is_boundary(_t))
                    return false;
                }
                else
                {
                    if (cut_mesh.is_boundary(_t))
                    return true;
                    else
                    {
                        auto _t_opp = switch_tuple(_t, PrimitiveType::Face);
                        auto _t_opp_cut = cut_mesh.switch_tuple(_t_opp, PrimitiveType::Face);

                        return _t_opp.same_ids(_t_opp_cut);
                    }
                }
                return true;
            };
            for (long i = 0; i < capacity(PrimitiveType::Face); ++i)
            {
                auto t = face_tuple_from_id(i);
                auto tn = switch_tuple(switch_tuple(t, PrimitiveType::Vertex), PrimitiveType::Edge);
                auto tnn = switch_tuple(t, PrimitiveType::Edge);
                
                if (!check_tuple(t) || !check_tuple(tn) || !check_tuple(tnn))
                {
                    return false;
                }
            }
        }
        return true;
    }

    // bool SyncTriMesh::sync_split_edge(const Tuple& t)
    // {
    //     if (!is_valid(t) || !cut_mesh.is_valid(t))
    //     {
    //         std::cout << "error: tuple is not valid" << std::endl;
    //         return false;
    //     }
    //     if (is_boundary(t))
    //     {
    //         split_edge(t);
    //         cut_mesh.split_edge(t);
    //     }
    //     else
    //     {
    //         auto t_opp = switch_tuple(t, PrimitiveType::Face).value();
    //         split_edge(t);
    //         cut_mesh.split_edge(t);
    //         cut_mesh.split_edge(t_opp);
    //     }

    //     // TODO: returns from split_edge default
    //     return true;
    // }

    // bool SyncTriMesh::sync_collpase_edge(const Tuple& t)
    // {
    //     if (!is_valid(t) || !cut_mesh.is_valid(t))
    //     {
    //         std::cout << "error: tuple is not valid" << std::endl;
    //         return false;
    //     }
    //     if (is_boundary(t))
    //     {
    //         if (!SimplicialComplex::link_cond(*this, t) || !SimplicialComplex::link_cond(cut_mesh, t))
    //         {
    //             std::cout << "error: link condition is not satisfied" << std::endl;
    //             return false;
    //         }

    //         collapse_edge(t);
    //         cut_mesh.collapse_edge(t);
    //     }
    //     else
    //     {
    //         auto t_opp = switch_tuple(t, PrimitiveType::Face).value();
    //         collapse_edge(t);
    //         cut_mesh.collapse_edge(t);
    //         cut_mesh.collapse_edge(t_opp);
    //     }

    //     // TODO: returns from collapse_edge default
    //     return true;
    // }
}// namespace wmtk