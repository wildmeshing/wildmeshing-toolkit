#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json cdt_opt_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": ["input", "output"],
        "optional": [
          "root",
          "report",
          "envelope_size",
          "length_rel",
          "target_max_amips",
          "max_passes"
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
    "pointer": "/target_max_amips",
    "type": "float",
    "default": 50
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
