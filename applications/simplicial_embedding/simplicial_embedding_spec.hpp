namespace {

nlohmann::json simplicial_embedding_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["input", "tag_name", "output"],
    "optional": ["tag_value", "embed_interfaces", "root", "report"]
  },
  {
    "pointer": "/input",
    "type": "string"
  },
  {
    "pointer": "/tag_name",
    "type": "string"
  },
  {
    "pointer": "/tag_value",
    "type": "int",
    "default": 0
  },
  {
    "pointer": "/embed_interfaces",
    "type": "bool",
    "default": false
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