#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json delaunay_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": ["bg_mesh", "tri_mesh", "output"],
        "optional": [
          "root",
          "report",
          "round",
          "track_submeshes",
          "make_child_free",
          "bg_pos_attr_name",
          "tri_pos_attr_name"
        ]
},
{
    "pointer": "/bg_mesh",
    "type": "string"
},
{
    "pointer": "/tri_mesh",
    "type": "string"
},
{
    "pointer": "/round",
    "type": "bool",
    "default": true
},
{
    "pointer": "/track_submeshes",
    "type": "bool",
    "default": true
},
{
    "pointer": "/make_child_free",
    "type": "bool",
    "default": false
},
{
    "pointer": "/bg_pos_attr_name",
    "type": "string",
    "default": "vertices"
},
{
    "pointer": "/tri_pos_attr_name",
    "type": "string",
    "default": "vertices"
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
