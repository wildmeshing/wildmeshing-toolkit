#pragma once
#include "TriMesh.hpp"

namespace wmtk {
    class SyncTriMesh : public TriMesh
    {
        public:
        std::vector<TriMesh> cut_meshes;

        void initizalize(Eigen::Ref<const RowVectors3l> F, std::vector<Eigen::Ref<const RowVectors3l>> F_cuts);

        // check if cut_mesh is a valid cut_mesh of this mesh
        bool is_cut_meshes_valid() const;

        // bool sync_split_edge(const Tuple& t);
        // bool sync_collpase_edge(const Tuple& t);
        
        // TODO: sync for other operations
        bool sync_edge_operations(const Tuple& t)
        {
            return true;
        };

        bool sync_vertex_operations(const Tuple& t)
        {
            return true;
        };
    };
}// namespace wmtk