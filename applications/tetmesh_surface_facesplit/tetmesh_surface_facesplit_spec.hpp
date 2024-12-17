#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json tetmesh_surface_facesplit_spec = R"(
[
    {
        "pointer": "/",
        "type": "object",
        "required": ["input", "output"],
        "optional": [
          "root",
          "report"
        ]
},
{
    "pointer": "/input",
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
