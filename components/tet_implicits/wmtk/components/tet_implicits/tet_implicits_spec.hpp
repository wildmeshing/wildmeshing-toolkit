#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json tet_implicits_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["application", "input", "operation"],
    "optional": [
      "output",
      "input_tags",
      "output_tags",
      "num_threads",
      "eps_rel",
      "eps",
      "d_rel",
      "d",
      "preserve_topology",
      "write_vtu",
      "log_file",
      "report",
      "DEBUG_output",
      "DEBUG_sanity_checks"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": ["tet_implicits"],
    "doc": "Application name must be tet_implicits."
  },
  {
    "pointer": "/input",
    "type": "list",
    "doc": "List of triangular input meshes.",
    "min": 1,
    "max": 1
  },
  {
    "pointer": "/input/*",
    "type": "string",
    "doc": "Triangular input mesh."
  },
  {
    "pointer": "/operation",
    "type": "string",
    "options": ["separate", "tight_seal"],
    "doc": "The operation to be performed."
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "out",
    "doc": "Output file name (without extension)."
  },
  {
    "pointer": "/input_tags",
    "type": "list",
    "default": [],
    "doc": "Input tags for the operation."
  },
  {
    "pointer": "/input_tags/*",
    "type": "list",
    "min": 2,
    "max": 2,
    "doc": "Tags are given by the tag_attribute ID and the ID within that tag_attribute. For example, if tag_attributes are [tag_0, tag_1], then the tag 2 in tag_0 is [0,2]."
  },
  {
    "pointer": "/input_tags/*/*",
    "type": "int"
  },
  {
    "pointer": "/output_tags",
    "type": "list",
    "default": [],
    "doc": "Output tags for the operation."
  },
  {
    "pointer": "/output_tags/*",
    "type": "list",
    "min": 2,
    "max": 2,
    "doc": "Tags are given by the tag_attribute ID and the ID within that tag_attribute. For example, if tag_attributes are [tag_0, tag_1], then the tag 2 in tag_0 is [0,2]."
  },
  {
    "pointer": "/output_tags/*/*",
    "type": "int"
  },
  {
    "pointer": "/num_threads",
    "type": "int",
    "default": 0,
    "doc": "Number of threads used by the application"
  },
  {
    "pointer": "/eps_rel",
    "type": "float",
    "default": 2e-3,
    "doc": "Envelope thickness relative to the bounding box"
  },
  {
    "pointer": "/eps",
    "type": "float",
    "default": -1,
    "doc": "Absolute envelope thickness. If this value is negative, the relative envelope thickness is used to compute the absolute one."
  },
  {
    "pointer": "/d_rel",
    "type": "float",
    "default": 5e-2,
    "doc": "Variable d relative to the bounding box"
  },
  {
    "pointer": "/d",
    "type": "float",
    "default": -1,
    "doc": "Variable d"
  },
  {
    "pointer": "/preserve_topology",
    "type": "bool",
    "default": true,
    "doc": "Preserve topology of input."
  },
  {
    "pointer": "/write_vtu",
    "type": "bool",
    "default": false,
    "doc": "Write not just MSH but also VTU output."
  },
  {
    "pointer": "/log_file",
    "type": "string",
    "default": "",
    "doc": "Logs are not just printed on the terminal but also saved in this file."
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": "",
    "doc": "A JSON file that stores information about the result and the method execution, e.g., runtime."
  },
  {
    "pointer": "/DEBUG_output",
    "type": "bool",
    "default": false,
    "doc": "Write the mesh as debug_{}.vtu after every operation."
  },
  {
    "pointer": "/DEBUG_sanity_checks",
    "type": "bool",
    "default": false,
    "doc": "Perform sanity checks after every operation. This can be very slow and should only be used for debugging."
  }
]
)"_json;

}