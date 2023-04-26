#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriMeshOperation.h>


namespace wmtk {

    // Fast prototyping using TriMesh
    //
    // 
    // TriMeshEasy easy_mesh;
    // AttributeCollection<Eigen::Vector3d> p;
    // easy_mesh.vertex_attrs = p;
    // easy_mesh.split_edge_after = [&](ExecuteReturnData& ret_data) {
    //    const Tuple& t = ret_data.tuple;
    //    const Tuple& c = t.switch_vertex(easy_mesh);;
    //    const Tuple& ot = t.switch_vertex(easy_mesh).switch_edge(easy_mesh).switch_face(easy_mesh).switch_vertex(easy_mesh);
    //
    //    
    //    p[c.vid(easy_mesh)] = .5 * (p[t.vid(easy_mesh)] + p[ot.vid(easy_mesh)]);
    //    
    // };
    // ExecutePass<TriMeshEasy, ExecutionPolicy::kSeq> exec; 
    // for(const auto& op_ptr: easy_mesh.getOperations()) { exec.add_operation(op_ptr); }
    //

class TriMeshEasy : public TriMesh
{
public:
    using ExecuteReturnData = TriMeshOperation::ExecuteReturnData;
    std::vector<std::shared_ptr<TriMeshOperation>> getOperations() const;


    std::function<bool(const Tuple&)> collapse_edge_before;
    std::function<bool(ExecuteReturnData&)> collapse_edge_after;
    std::function<bool(const Tuple&)> split_edge_before;
    std::function<bool(ExecuteReturnData&)> split_edge_after;
    std::function<bool(const Tuple&)> swap_edge_before;
    std::function<bool(ExecuteReturnData&)> swap_edge_after;
    std::function<bool(const Tuple&)> vertex_smooth_before;
    std::function<bool(ExecuteReturnData&)> vertex_smooth_after;
    std::function<bool(ExecuteReturnData&)> invariants;
};
} // namespace wmtk
