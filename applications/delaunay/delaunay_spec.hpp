namespace {

nlohmann::json delaunay_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["input"],
    "optional": ["output", "report"]
  },
  {
    "pointer": "/input",
    "type": "string"
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": ""
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": ""
  }
]
)"_json;

}