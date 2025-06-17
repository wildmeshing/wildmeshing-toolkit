#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json simplicial_embedding_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["application", "input"],
    "optional": ["output"]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": ["simplicial_embedding"],
    "doc": "Application name must be simplicial_embedding."
  },
  {
    "pointer": "/input",
    "type": "string",
    "doc": "Triangular or tetrahedral input mesh."
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "out",
    "doc": "Output file name (without extension)."
  }
]
)"_json;

}