#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json longest_edge_split_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": [
      "input",
      "output"
    ],
    "optional": [
      "length_rel",
      "lock_boundary",
      "report",
      "input_path",
      "use_multimesh"
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
    "pointer": "/length_rel",
    "type": "float",
    "default": 0.1,
    "doc": "The desired edge length relative to the AABB."
  },
  {
    "pointer": "/lock_boundary",
    "type": "bool",
    "default": false,
    "doc": "Are boundary vertices allowed to be split?"
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": ""
  },
  {
    "pointer": "/input_path",
    "type": "string",
    "default": ""
  },
  {
    "pointer": "/use_multimesh",
    "type": "string",
    "default": "none",
    "options": [
      "none",
      "interior",
      "boundary"
    ]
  }
]
)"_json;

}