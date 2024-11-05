#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json tetwild_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": ["input", "output"],
        "optional": [
          "root",
          "report",
          "envelope_size",
          "target_edge_length",
          "skip_simplification",
          "target_max_amips",
          "max_passes",
          "intermediate_output",
          "skip_split",
          "skip_collapse",
          "skip_swap",
          "skip_smooth"
        ]
},
{
    "pointer": "/input",
    "type": "string"
},
{
    "pointer": "/envelope_size",
    "type": "float",
    "default": 0.001
},
{
    "pointer": "/skip_simplification",
    "type": "bool",
    "default": false
},
{
    "pointer": "/intermediate_output",
    "type": "bool",
    "default": false
},
{
    "pointer": "/target_max_amips",
    "type": "float",
    "default": 10.0
},
{
    "pointer": "/target_edge_length",
    "type": "float",
    "default": 0.05
},
{
    "pointer": "/max_passes",
    "type": "int",
    "default": 10
},
{
    "pointer": "/skip_split",
    "type": "bool",
    "default": false
},
{
    "pointer": "/skip_collapse",
    "type": "bool",
    "default": false
},
{
    "pointer": "/skip_swap",
    "type": "bool",
    "default": false
},
{
    "pointer": "/skip_smooth",
    "type": "bool",
    "default": false
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