#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json image_simulation_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["application", "input"],
    "optional": [
      "output",
      "skip_simplify",
      "use_sample_envelope",
      "num_threads",
      "max_iterations",
      "eps_rel",
      "length_rel",
      "stop_energy",
      "write_vtu"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": ["image_simulation"],
    "doc": "Application name must be image_simulation."
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
    "pointer": "/write_vtu",
    "type": "bool",
    "default": false,
    "doc": "Write not just MSH but also VTU output."
  }
]
)"_json;

}