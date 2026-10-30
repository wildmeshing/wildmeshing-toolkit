"""Generate the component reference from the components' *_spec.json files.

For every components/<name>/wmtk/components/<name>/<name>_spec.json, with docs/components.yml for
how to present it, this writes

    reference/<name>.md             the option reference page
    reference/<name>_defaults.json  every top-level option that has a default, with it
    reference/index.md              the table of all components
    reference/SUMMARY.md            the reference's navigation (mkdocs-literate-nav)
    schemas/<name>.json             a JSON Schema of the options, for editors
    schemas/wmtk_app.json           one schema for any wmtk_app file, by its "application"

mkdocs-gen-files runs this on every build (mkdocs.yml); nothing it writes is committed. Run it
directly to check the specs, as CI does:

    python docs/tools/gen_reference.py --check      # every option documented, components.yml in sync
    python docs/tools/gen_reference.py --out DIR    # write the files to DIR instead
"""

import argparse
import json
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
SPEC_SOURCE = "https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/"

# Options every component accepts that control the run rather than the algorithm. The reference
# lists them last.
RUN_CONTROL = ["application", "input_dir", "num_threads", "log_file", "report", "throw_on_fail"]

# Option name prefixes for expert options: the reference folds each group away.
EXPERT_GROUPS = [
    ("DEBUG_", "Debug options", "Diagnostics: slow checks and extra output files."),
    ("EXPERIMENTAL_", "Experimental options", "Under development: they may change or go away."),
    ("TESTING_", "Testing options", "For the test suites."),
]

TYPE_NAMES = {"string": "string", "bool": "bool", "int": "int", "float": "float",
              "list": "list", "object": "object"}
SCHEMA_TYPES = {"string": "string", "bool": "boolean", "int": "integer", "float": "number",
                "list": "array", "object": "object"}


def find_specs():
    """{name: path} of every component spec."""
    specs = {}
    for path in sorted(ROOT.glob("components/*/wmtk/components/*/*_spec.json")):
        name = path.name[: -len("_spec.json")]
        if path.parent.name == name:
            specs[name] = path
    return specs


class Spec:
    """A component spec: its rules by pointer."""

    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.rules = json.loads(path.read_text())
        self.by_pointer = {}
        for rule in self.rules:
            self.by_pointer.setdefault(rule["pointer"], []).append(rule)
        self.root = self.by_pointer["/"][0]

    def doc(self):
        return self.root.get("doc", "").strip()

    def top_level(self):
        """Top-level option names, in the order the spec lists them."""
        names = []
        for rule in self.rules:
            parts = rule["pointer"].split("/")[1:]
            if len(parts) == 1 and parts[0] and parts[0] not in names:
                names.append(parts[0])
        return names

    def required(self):
        return list(self.root.get("required", []))

    def children(self, pointer):
        """The rules one level below pointer, by child name ('*' for list items)."""
        prefix = pointer.rstrip("/") + "/"
        out = {}
        for p, rules in self.by_pointer.items():
            child = p[len(prefix):]
            if p.startswith(prefix) and child and "/" not in child:
                out[child] = rules
        return out


def escape(text):
    """Markdown-safe text: < > and * outside code spans would be read as HTML and emphasis."""
    parts = re.split(r"(`[^`]*`)", text)
    for i in range(0, len(parts), 2):
        parts[i] = (parts[i].replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                    .replace("*", r"\*"))
    return "".join(parts)


def type_label(spec, pointer, rule):
    """'float', 'list of string', 'list of objects', ..."""
    if rule["type"] != "list":
        return TYPE_NAMES[rule["type"]]
    items = spec.by_pointer.get(pointer.rstrip("/") + "/*", [])
    if not items:
        return "list"
    inner = " or ".join(dict.fromkeys(
        type_label(spec, pointer.rstrip("/") + "/*", r) for r in items))
    return "list of " + ("objects" if inner == "object" else inner)


def meta_line(spec, pointer, rule, required):
    bits = [f"**{type_label(spec, pointer, rule)}**"]
    if required:
        bits.append("required")
    if "default" in rule:
        bits.append(f"default `{json.dumps(rule['default'])}`")
    if "options" in rule:
        bits.append("one of " + ", ".join(f"`{o}`" for o in rule["options"]))
    if "min" in rule:
        bits.append(f"min `{rule['min']}`")
    if "max" in rule:
        bits.append(f"max `{rule['max']}`")
    return " · ".join(bits)


def object_fields(spec, pointer, indent=""):
    """Bullets for the fields of the objects at pointer (a list item or an object option)."""
    lines = []
    for rule in spec.by_pointer.get(pointer, []):
        if rule["type"] != "object":
            continue
        required = set(rule.get("required", []))
        for field, rules in spec.children(pointer).items():
            for r in rules:
                meta = meta_line(spec, f"{pointer}/{field}", r, field in required)
                doc = escape(r.get("doc", "").strip())
                lines.append(f"{indent}- `{field}`: {meta}. {doc}".rstrip())
    return lines


def option_body(spec, name, required):
    """The lines describing option name: one block per alternative rule."""
    pointer = "/" + name
    lines = []
    for rule in spec.by_pointer[pointer]:
        lines.append(meta_line(spec, pointer, rule, required))
        lines.append("")
        if rule.get("doc", "").strip():
            lines.append(escape(rule["doc"].strip()))
            lines.append("")
        items = pointer + "/*"
        if rule["type"] == "list" and items in spec.by_pointer:
            for item in spec.by_pointer[items]:
                if item.get("doc", "").strip():
                    lines.append(f"Each entry: {escape(item['doc'].strip())}")
                    lines.append("")
            fields = object_fields(spec, items)
            if fields:
                lines.append("Each entry is an object with:")
                lines.append("")
                lines += fields + [""]
        elif rule["type"] == "object":
            fields = object_fields(spec, pointer)
            if fields:
                lines += fields + [""]
    return lines


def defaults(spec):
    out = {}
    for name in spec.top_level():
        for rule in spec.by_pointer["/" + name]:
            if "default" in rule:
                out[name] = rule["default"]
                break
    return out


def first_sentence(text):
    text = re.sub(r"\s+", " ", text).strip()
    m = re.match(r"(.+?[.!?])(\s|$)", text)
    return m.group(1) if m else text


def reference_page(spec, meta, groups):
    name = spec.name
    title = meta.get("title", name)
    lines = [f"# {title} (`{name}`)", ""]
    if meta.get("group") == "wip":
        lines += ['!!! warning "Work in progress"',
                  "    This component is under development. Its options may change, and it has "
                  "no tutorial yet.", ""]
    if spec.doc():
        lines += [escape(spec.doc()), ""]
    links = []
    if meta.get("tutorial"):
        links.append(f"[Tutorial](../{meta['tutorial']})")
    rel = spec.path.relative_to(ROOT).as_posix()
    links += [f"[Spec source]({SPEC_SOURCE}{rel})",
              f"[Defaults]({name}_defaults.json)",
              f"[JSON Schema](../schemas/{name}.json)"]
    lines += [" · ".join(links), ""]
    if meta.get("papers"):
        lines += ["Cite: " + "; ".join(meta["papers"]), ""]

    options = spec.top_level()
    required = spec.required()
    key = [o for o in meta.get("key_options", []) if o in options and o not in required]
    expert = {prefix: [o for o in options if o.startswith(prefix)] for prefix, _, _ in EXPERT_GROUPS}
    expert_names = {o for group in expert.values() for o in group}
    run = [o for o in RUN_CONTROL if o in options and o not in required]
    rest = [o for o in options
            if o not in required and o not in key and o not in expert_names and o not in run]

    def section(heading, names, req):
        if not names:
            return []
        out = [f"## {heading}", ""]
        for o in names:
            out += [f"### `{o}` {{ #{o} }}", ""] + option_body(spec, o, req)
        return out

    lines += section("Required", required, True)
    lines += section("Key options", key, False)
    lines += section("Other options", rest, False)
    lines += section("Run control", run, False)
    for prefix, heading, blurb in EXPERT_GROUPS:
        names = expert[prefix]
        if not names:
            continue
        lines += [f"## {heading}", "", f'??? note "{blurb} ({len(names)})"', ""]
        for o in names:
            body = option_body(spec, o, False)
            lines.append(f'    <a id="{o}"></a>**`{o}`** · ' + body[0])
            lines += [("    " + b) if b else "" for b in body[1:]]
    return "\n".join(lines).rstrip() + "\n"


def index_page(specs, meta, groups):
    lines = ["# Components", "",
             "Every component runs from a JSON file (`wmtk_app -j file.json`), from Python "
             "(`wildmeshing.<name>(options)`) and from C++ (`wmtk::components::<name>::<name>`). "
             "Each page lists every option of one component, generated from its "
             "`<name>_spec.json`.", ""]
    for gid, gtitle in groups.items():
        names = [n for n in specs if meta[n].get("group") == gid]
        if not names:
            continue
        lines += [f"## {gtitle}", "", "| Component | Description | Tutorial |", "|---|---|---|"]
        for n in names:
            m = meta[n]
            tut = f"[tutorial](../{m['tutorial']})" if m.get("tutorial") else "—"
            desc = escape(first_sentence(specs[n].doc())) if specs[n].doc() else ""
            lines.append(f"| [{m.get('title', n)}]({n}.md) (`{n}`) | {desc} | {tut} |")
        lines.append("")
    return "\n".join(lines)


def summary_page(specs, meta, groups):
    lines = ["* [Overview](index.md)"]
    for gid, gtitle in groups.items():
        names = [n for n in specs if meta[n].get("group") == gid]
        if names:
            lines.append(f"* {gtitle}")
            lines += [f"    * [{meta[n].get('title', n)}]({n}.md)" for n in names]
    return "\n".join(lines) + "\n"


def schema_of(spec, pointer):
    rules = spec.by_pointer.get(pointer, [])
    alternatives = [schema_of_rule(spec, pointer, r) for r in rules]
    if not alternatives:
        return {}
    return alternatives[0] if len(alternatives) == 1 else {"anyOf": alternatives}


def schema_of_rule(spec, pointer, rule):
    s = {"type": SCHEMA_TYPES[rule["type"]]}
    if rule.get("doc", "").strip():
        s["description"] = rule["doc"].strip()
    if "default" in rule:
        s["default"] = rule["default"]
    if "options" in rule:
        s["enum"] = rule["options"]
    if "min" in rule:
        s["minimum"] = rule["min"]
    if "max" in rule:
        s["maximum"] = rule["max"]
    if rule["type"] == "list":
        items = pointer.rstrip("/") + "/*"
        if items in spec.by_pointer:
            s["items"] = schema_of(spec, items)
    if rule["type"] == "object":
        children = spec.children(pointer)
        s["properties"] = {c: schema_of(spec, pointer.rstrip("/") + "/" + c)
                           for c in children if c != "*"}
        if rule.get("required"):
            s["required"] = list(rule["required"])
    return s


def component_schema(spec, base_url):
    s = schema_of(spec, "/")
    s["$schema"] = "https://json-schema.org/draft/2020-12/schema"
    s["$id"] = f"{base_url}schemas/{spec.name}.json"
    s["title"] = spec.name
    # wmtk_app accepts the key that names this schema, and ignores it.
    s.setdefault("properties", {})["$schema"] = {"type": "string"}
    # Only topological_offset refuses an option its spec does not list; the others ignore it,
    # so a misspelled option silently keeps its default. The schema flags it either way.
    s["additionalProperties"] = False
    return s


def app_schema(specs, base_url):
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": f"{base_url}schemas/wmtk_app.json",
        "title": "wmtk_app",
        "description": "A wmtk_app input file: its \"application\" names the component.",
        "type": "object",
        "required": ["application"],
        "oneOf": [{"$ref": f"{n}.json"} for n in specs],
    }


def check(specs, meta, groups):
    errors = []
    for name in specs:
        if name not in meta:
            errors.append(f"{name}: no entry in docs/components.yml")
    for name, m in meta.items():
        if name not in specs:
            errors.append(f"docs/components.yml: {name} has no spec")
            continue
        if m.get("group") not in groups:
            errors.append(f"docs/components.yml: {name}: unknown group {m.get('group')!r}")
        options = specs[name].top_level()
        for o in m.get("key_options", []):
            if o not in options:
                errors.append(f"docs/components.yml: {name}: key option {o!r} is not in its spec")
        if m.get("tutorial") and not (ROOT / "docs" / m["tutorial"]).exists():
            errors.append(f"docs/components.yml: {name}: tutorial {m['tutorial']} does not exist")
    for name, spec in specs.items():
        rel = spec.path.relative_to(ROOT)
        if not spec.doc():
            errors.append(f"{rel}: the root rule (\"/\") has no doc")
        for rule in spec.rules:
            if rule["pointer"] != "/" and not rule.get("doc", "").strip():
                errors.append(f"{rel}: {rule['pointer']} has no doc")
    return errors


def generate(write, base_url):
    meta_file = yaml.safe_load((ROOT / "docs" / "components.yml").read_text())
    groups, meta = meta_file["groups"], meta_file["components"]
    found = find_specs()
    # In components.yml's order, then any spec it does not list (--check reports those).
    order = [n for n in meta if n in found] + [n for n in found if n not in meta]
    specs = {n: Spec(n, found[n]) for n in order}
    for n in specs:
        meta.setdefault(n, {"title": n, "group": "wip"})
    for name, spec in specs.items():
        write(f"reference/{name}.md", reference_page(spec, meta[name], groups))
        write(f"reference/{name}_defaults.json", json.dumps(defaults(spec), indent=2) + "\n")
        write(f"schemas/{name}.json", json.dumps(component_schema(spec, base_url), indent=2) + "\n")
    write("reference/index.md", index_page(specs, meta, groups))
    write("reference/SUMMARY.md", summary_page(specs, meta, groups))
    write("schemas/wmtk_app.json", json.dumps(app_schema(specs, base_url), indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--check", action="store_true", help="check the specs and components.yml")
    p.add_argument("--out", type=Path, help="write the generated files to this directory")
    args = p.parse_args()
    if args.check:
        meta_file = yaml.safe_load((ROOT / "docs" / "components.yml").read_text())
        specs = {n: Spec(n, pth) for n, pth in find_specs().items()}
        errors = check(specs, meta_file["components"], meta_file["groups"])
        for e in errors:
            print(e, file=sys.stderr)
        print(f"{len(specs)} specs, {len(errors)} problems")
        sys.exit(1 if errors else 0)
    if args.out:
        def write(rel, text):
            path = args.out / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        generate(write, "https://wildmeshing.github.io/wildmeshing-toolkit/")
        return
    p.error("run by mkdocs-gen-files, or with --check or --out")


if __name__ == "__main__":
    main()
else:
    # Run by mkdocs-gen-files.
    import mkdocs_gen_files

    def _write(rel, text):
        with mkdocs_gen_files.open(rel, "w") as f:
            f.write(text)

    _config = mkdocs_gen_files.config
    generate(_write, _config.site_url or "")
