#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json delaunay_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": ["input", "output"],
        "optional": [
          "root",
          "report",
          "add_box",
          "box_scale",
          "add_grid",
          "grid_spacing",
          "min_dist",
          "remove_duplicates",
          "output_pos_attr_name"
        ]
},
{
    "pointer": "/input",
    "type": "string"
},
{
    "pointer": "/add_box",
    "type": "bool",
    "default": false
},
{
    "pointer": "/box_scale",
    "type": "float",
    "default": 1
},
{
    "pointer": "/add_grid",
    "type": "bool",
    "default": false
},
{
    "pointer": "/grid_spacing",
    "type": "float",
    "default": 0.1
},
{
    "pointer": "/min_dist",
    "type": "float",
    "default": -1
},
{
    "pointer": "/remove_duplicates",
    "type": "bool",
    "default": false
},
{
    "pointer": "/output_pos_attr_name",
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
