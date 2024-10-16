#pragma once
#include <nlohmann/json_fwd.hpp>
#include <memory>
namespace wmtk {
    class Mesh;
}


//
// {
//      input_params
//      "file": "a.msh"
//
//      and one of:
//      "boundary": {
//      }
//
//      OR
//
//
//      // creates a new mesh whose topology comes from fusing every vertex according to an integer grid according to some epsilon
//      "fusion": {
//          "attribute": {
//           // attribute spec
//              "name": "vertices"
//              "simplex": 0//optional
//              "type": "double"//optional
//          }
//          "axes": [true,false,true] 
//          "eps": 1e-10
//          
//
//      OR
//
//      // returns multimesh with a as the root, b as child
//      "facet_bijection": { 
//          
//          // input spec
//          input params for child mesh like
//          "file": "b.msh"
//      }
// }
//

std::shared_ptr<wmtk::Mesh> make_multimesh(wmtk::Mesh& m, const nlohmann::json& multimesh_config);

