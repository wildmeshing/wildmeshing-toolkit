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
          "max_amips",
          "max_passes",
          "intermediate_output"
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
    "pointer": "/length_rel",
    "type": "float",
    "default": 0.1
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
    "pointer": "/max_amips",
    "type": "float",
    "default": 10.0
},
{
    "pointer": "/max_passes",
    "type": "int",
    "default": 10
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