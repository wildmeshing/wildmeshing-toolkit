#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json c1_meshing_split_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": [
          "tetmesh", 
          "surface_mesh", 
          "uv_mesh", 
          "tet_surface_map", 
          "parametrization_edges", 
          "adjacent_cone_edges", 
          "output"
        ],
        "optional": [
          "root",
          "report"
        ]
},
{
    "pointer": "/tetmesh",
    "type": "string"
},
{
    "pointer": "/surface_mesh",
    "type": "string"
},
{
    "pointer": "/tet_surface_map",
    "type": "string"
},
{
    "pointer": "/uv_mesh",
    "type": "string"
},
{
    "pointer": "/parametrization_edges",
    "type": "string"
},
{
    "pointer": "/adjacent_cone_edges",
    "type": "string"
},
{
    "pointer": "/output",
    "type": "string"
},
{
    "pointer": "/report",
    "type": "string",
    "default": ""
},
{
    "pointer": "/root",
    "type": "string",
    "default": ""
}
]
)"_json;

}
