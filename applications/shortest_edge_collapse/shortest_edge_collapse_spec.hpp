#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json shortest_edge_collapse_spec = R"(
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
      "envelope_size",
      "lock_boundary",
      "report",
      "input_path",
      "use_multimesh",
      "check_inversion"
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
    "pointer": "/envelope_size",
    "type": "float",
    "default": 0.001,
    "doc": "The envelope size relative to the AABB. Set this to a negative value to deactivate the envelope."
  },
  {
    "pointer": "/lock_boundary",
    "type": "bool",
    "default": false,
    "doc": "Are boundary vertices allowed to be collapsed?"
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
    "pointer": "/check_inversion",
    "type": "bool",
    "default": false
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
