#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriMeshOperation.h>


namespace wmtk {

    // Fast prototyping using TriMesh
    //
    // 
    // TriMeshFun fun_mesh;
    // AttributeCollection<Eigen::Vector3d> p;
    // fun_mesh.vertex_attrs = p;
    // fun_mesh.split_edge_after = [&](const Tuple& ret_data) {
    //    const Tuple& t = ret_data.tuple;
    //    const Tuple& c = t.switch_vertex(fun_mesh);;
    //    const Tuple& ot = t.switch_vertex(fun_mesh).switch_edge(fun_mesh).switch_face(fun_mesh).switch_vertex(fun_mesh);
    //
    //    
    //    p[c.vid(fun_mesh)] = .5 * (p[t.vid(fun_mesh)] + p[ot.vid(fun_mesh)]);
    //    
    // };
    // ExecutePass<TriMeshFun, ExecutionPolicy::kSeq> exec; 
    // for(const auto& op_ptr: fun_mesh.get_operations()) { exec.add_operation(op_ptr); }
    //

class TriMeshFun : public TriMesh
{
public:
    using Tuple = TriMesh::Tuple;
    std::map<std::string,std::shared_ptr<TriMeshOperation>> get_operations() const override;


    std::function<bool(const Tuple&)> collapse_edge_before;
    std::function<bool(const Tuple&)> collapse_edge_after;
    std::function<bool(const Tuple&)> split_edge_before;
    std::function<bool(const Tuple&)> split_edge_after;
    std::function<bool(const Tuple&)> swap_edge_before;
    std::function<bool(const Tuple&)> swap_edge_after;
    std::function<bool(const Tuple&)> vertex_smooth_before;
    std::function<bool(const Tuple&)> vertex_smooth_after;
    std::function<bool(const std::vector<Tuple>&)> fun_invariants;
    bool invariants(const TriMeshOperation& op) override;
};
} // namespace wmtk
