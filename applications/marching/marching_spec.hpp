namespace {

nlohmann::json delaunay_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["input", "output"],
    "optional": ["input_path", "report"]
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
    "pointer": "/input_path",
    "type": "string",
    "default": ""
  }
]
)"_json;

}