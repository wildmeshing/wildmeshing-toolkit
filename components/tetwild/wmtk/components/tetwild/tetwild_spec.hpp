#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json tetwild_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["application", "input"],
    "optional": [
      "output",
      "skip_simplify",
      "use_sample_envelope",
      "use_legacy_code",
      "num_threads",
      "max_iterations",
      "filter",
      "eps_rel",
      "length_rel",
      "stop_energy",
      "preserve_topology",
      "throw_on_fail",
      "log_file",
      "report",
      "DEBUG_output",
      "DEBUG_sanity_checks",
      "DEBUG_hausdorff"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": ["tetwild"],
    "doc": "Application name must be tetwild."
  },
  {
    "pointer": "/input",
    "type": "list",
    "doc": "List of triangular input meshes.",
    "min": 1
  },
  {
    "pointer": "/input/*",
    "type": "string",
    "doc": "Triangular input mesh."
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "out",
    "doc": "Output file name (without extension)."
  },
  {
    "pointer": "/skip_simplify",
    "type": "bool",
    "default": false,
    "doc": "If true, input simplification will be skipped."
  },
  {
    "pointer": "/use_sample_envelope",
    "type": "bool",
    "default": false,
    "doc": "Use sample envelope instead of exact one."
  },
  {
    "pointer": "/use_legacy_code",
    "type": "bool",
    "default": false,
    "doc": "Use the original TetWild code for the mesh improvement. This only works with 'use_sample_envelope'!"
  },
  {
    "pointer": "/num_threads",
    "type": "int",
    "default": 0,
    "doc": "Number of threads used by the application"
  },
  {
    "pointer": "/max_iterations",
    "type": "int",
    "default": 10,
    "doc": "Maximum iterations before stopping."
  },
  {
    "pointer": "/filter",
    "type": "string",
    "options": ["flood", "input", "tracked", "none"],
    "default": "tracked",
    "doc": "Remove the outside region based on different criteria. 'flood': flood fill. 'input': winding number w.r.t. the input. 'tracked': winding number w.r.t. the tracked surface. 'none': Do not filter. Flood fill only works if the input is closed. Otherwise, it results in an empty mesh. Filtering w.r.t. the input might cause wrinkles along the surface as some tets might be falsely tagged. Filtering w.r.t. the tracked surface can lead to missing pieces if the input consists of multiple components."
  },
  {
    "pointer": "/eps_rel",
    "type": "float",
    "default": 2e-3,
    "doc": "Envelope thickness relative to the bounding box"
  },
  {
    "pointer": "/length_rel",
    "type": "float",
    "default": 5e-2,
    "doc": "Target edge length relative to the bounding box"
  },
  {
    "pointer": "/stop_energy",
    "type": "float",
    "default": 10,
    "doc": "Target energy. If all tets have an energy below this, tetwild will stop."
  },
  {
    "pointer": "/preserve_topology",
    "type": "bool",
    "default": false,
    "doc": "Preserve the topology of the input surface."
  },
  {
    "pointer": "/throw_on_fail",
    "type": "bool",
    "default": false,
    "doc": "Throw exception if the output does not fulfil the desired criteria. No output will be generated."
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
  },
  {
    "pointer": "/DEBUG_hausdorff",
    "type": "bool",
    "default": false,
    "doc": "Sanity Check: Compute and report the Hausdorff distance of the output to the input. Should be always smaller than eps."
  }
]
)"_json;

}