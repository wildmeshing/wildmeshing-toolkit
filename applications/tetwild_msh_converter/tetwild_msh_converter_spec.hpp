#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json tetwild_msh_converter_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["input"],
    "optional": ["output", "report", "input_path"]
  },
  {
    "pointer": "/input",
    "type": "string",
    "doc": "Input MSH file"
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "",
    "doc": "Output VTU file. By default, the same as the input file name but with .vtu extension."
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": ""
  },
  {
    "pointer": "/input_path",
    "type": "string",
    "default": "",
    "doc": " The folder in which the input file is located. By default, this path will be set to the folder of the JSON spec file."
  }
]
)"_json;

}