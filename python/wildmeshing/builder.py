"""A pipeline builder: stack components as stages, edit their options, run, read each stage's log
and view each stage's result, in one polyscope window -- over wildmeshing.pipeline (the pipeline
file, its stage keys and cache) and wildmeshing.viewer (the view of each stage).

    python -m wildmeshing.builder [<pipeline.json>]

The "pipeline" window lists the inputs first, a table: name (editable; every reference follows a
rename), file path ("..." for the system's file dialog), "x"; its last row adds one. Then the
stages in run order, each with where its last result came from (memory, cache, save, run),
"edited" once it or a stage it reads has changed since, or "failed". A new stage is named
<n>_<component> until typed over, and reads the stage before it: its input that stage's mesh, its
envelope that stage's envelope; the first stage reads the inputs. A component stage calls one
component, on options edited as the component's spec types them (wm.spec: the spec the compiled
module holds, so the fields follow the installed build): checkboxes, numbers, choices, text,
lists item by item, JSON for objects. Only the options set are written to the pipeline; "other
options", folded, lists the others -- a filter, each with its default, its doc as its tooltip,
"+" to set it; EXPERIMENTAL_ and DEBUG_ ones in a group of their own. "ref" turns a value into a
reference to an earlier stage's output mesh, report value or option. "run" on a stage runs the
pipeline through that stage in a background thread -- the stages above it from memory or the
cache unless they changed; the bindings release the GIL, so the window keeps drawing -- and each
stage it runs appears in the "view" window, shown alone. The "log" window holds a stage's log,
live while it runs. Editing is off while a run goes on. "save" writes the pipeline file; its
paths stay relative to the folder the pipeline was loaded from (the current folder for a new one).

Needs polyscope (pip install -e ".[viewer]").
"""
import copy
import json
import os
import queue
import subprocess
import sys
import textwrap
import threading

import wildmeshing as wm

from . import viewer as _viewer
from .pipeline import Pipeline, StageFailed, _is_ref, _refs, _replace, check_name

ZERO = {"bool": False, "int": 0, "float": 0.0, "string": "", "list": [], "object": {}}
SCALARS = ("bool", "int", "float", "string")
RED = (0.9, 0.3, 0.3, 1.0)
MARGIN = 10  # between windows, as polyscope's own
LEFT = 305  # the width of polyscope's own windows, at ui scale 1
# The system's file dialog, in a process of its own: tkinter in this one would share macOS's one
# application object with polyscope's window.
PICK_FILE = """
import sys, tkinter, tkinter.filedialog
root = tkinter.Tk()
root.withdraw()
root.attributes("-topmost", True)
print(tkinter.filedialog.askopenfilename(parent=root, title="mesh file", initialdir=sys.argv[1]))
"""
RIGHT = 520  # the right column's width at start: "pipeline" and "view"


def _default(rule):
    """A new value for an option: its spec default, else its type's zero."""
    return copy.deepcopy(rule["default"] if "default" in rule else ZERO.get(rule.get("type"), ""))


def _parses(text):
    try:
        json.loads(text)
        return True
    except ValueError:
        return False


class Builder:
    """The pipeline being built, its runs and its windows. The methods that change the pipeline
    are the panel's buttons, callable without a window."""

    def __init__(self, pipeline, path=None):
        self.pipeline, self.path = pipeline, path or "pipeline.json"
        self.status = {}  # stage -> its last result's source, "edited" or "failed"
        self.logs = {}  # stage -> its log's path
        self.viewed = {}  # stage -> the key of the result the viewer shows
        self.queue = queue.Queue()
        self.thread = None
        self.message = ""
        self.log_stage = None
        self.log = {"path": None, "size": -1, "text": ""}
        self.new = {"kind": 0, "name": ""}  # the stage being added
        self.new_input = {"name": "", "file": ""}
        self.text = {}  # the JSON text of a field being edited
        self.specs = {}
        self.filters = {}  # stage -> the filter of its other options
        self.renaming = {}  # input -> the name being typed over it
        self.column = RIGHT  # the right column's width, as the pipeline window is dragged
        self.viewer = self.ps = self.psim = None  # set by run_window()

    def spec(self, component):
        """The component's option rules by pointer, its required and its optional options; the
        application name aside, which the memory route does not need."""
        if component not in self.specs:
            rules = {r["pointer"]: r for r in wm.spec(component)}
            root = rules.get("/", {})
            self.specs[component] = (rules,
                                     [k for k in root.get("required", []) if k != "application"],
                                     [k for k in root.get("optional", []) if k != "application"])
        return self.specs[component]

    def stage(self, name):
        return next(s for s in self.pipeline.stages if s["name"] == name)

    def names(self):
        """The inputs' and the stages' names: one set, each a name a reference can give."""
        return list(self.pipeline.inputs) + [s["name"] for s in self.pipeline.stages]

    def above(self, name):
        """What `name` can read: the inputs, then the stages above it."""
        names = self.names()
        return names[:names.index(name)]

    def readers(self, name):
        """The stages that read `name`, directly or through others."""
        out = {name}
        for s in self.pipeline.stages:
            if any(r["from"] in out for r in _refs(s.get("options", {}))):
                out.add(s["name"])
        return out - {name}

    def _edited(self, name):
        for n in {name} | self.readers(name):
            if n in self.status:
                self.status[n] = "edited"

    def rename(self, old, new):
        """Rename an input or a stage, and every reference to it. The stages that read it rerun:
        an output goes in under its name, which is in their keys."""
        if new == old:
            return
        check_name(new, self.names())
        inputs = self.pipeline.inputs
        if old in inputs:
            self.pipeline.inputs = {new if k == old else k: v for k, v in inputs.items()}
        else:
            self.stage(old)["name"] = new
        for s in self.pipeline.stages:
            s["options"] = _replace(s["options"],
                                    lambda r: {**r, "from": new} if r["from"] == old else r)
        for d in (self.pipeline.results, self.status, self.logs):
            if old in d:
                d[new] = d.pop(old)
        for n in self.readers(new):
            if n in self.status:
                self.status[n] = "edited"
        self.viewed.pop(old, None)
        result = self.pipeline.results.get(new)
        if self.viewer is not None and result is not None:
            shown = any(s["name"] == old and s["shown"] for s in self.viewer.stages)
            self.viewer.set_stage(new, result["outputs"], result["report"], self._input(new),
                                  shown=shown, replace=old)
            self.viewed[new] = result["key"]

    def add_input(self, name, file):
        check_name(name, self.names())
        self.pipeline.inputs[name] = file

    def add_stage(self, name, component):
        """Append a stage calling `component`: its required options at their defaults, then its
        references to what it most likely reads (_chained)."""
        check_name(name, self.names())
        rules, required, _ = self.spec(component)
        options = {k: _default(rules.get(f"/{k}", {})) for k in required}
        options.update(self._chained(component))
        self.pipeline.stages.append({"name": name, "component": component, "options": options})

    def _chained(self, component):
        """A new stage reads the stage before it -- the last one that returns a mesh, or has not
        run yet: a tracking run returns none -- its `input` that stage's mesh, its `envelope`
        that stage's envelope (unless it ran and returned none). With no stage before it, its
        `input` is the inputs -- all of them for a list, named by `input_names` when it has one;
        the last one otherwise. Only a required `input` is chained: an optional one means
        something else (tracking's: the surface a history starts from)."""
        rules, required, _ = self.spec(component)
        results = self.pipeline.results
        before = next((s["name"] for s in reversed(self.pipeline.stages) if s["name"] not in
                       results or "mesh" in results[s["name"]]["outputs"]), None)
        inputs = list(self.pipeline.inputs)
        out = {}
        if "input" in required and (before or inputs):
            listed = rules["/input"].get("type") == "list"
            if before:
                out["input"] = [{"from": before, "output": "mesh"}] if listed else \
                    {"from": before, "output": "mesh"}
            elif listed:
                out["input"] = [{"from": n, "output": "mesh"} for n in inputs]
                if "/input_names" in rules:
                    out["input_names"] = list(inputs)
            else:
                out["input"] = {"from": inputs[-1], "output": "mesh"}
        result = results.get(before)
        if "/envelope" in rules and before and (result is None or "envelope" in result["outputs"]):
            out["envelope"] = {"from": before, "output": "envelope"}
        return out

    def remove(self, name):
        """Remove an input or a stage that nothing reads."""
        readers = self.readers(name)
        if readers:
            raise ValueError(f"{name} is read by {', '.join(sorted(readers))}")
        if name in self.pipeline.inputs:
            del self.pipeline.inputs[name]
        else:
            self.pipeline.stages.remove(self.stage(name))
        for d in (self.pipeline.results, self.status, self.logs, self.viewed):
            d.pop(name, None)
        if self.viewer is not None:
            self.viewer.remove_stage(name)

    def set_file(self, name, file):
        self.pipeline.inputs[name] = file
        self._edited(name)

    def set_option(self, name, key, value):
        self.stage(name)["options"][key] = value
        self._edited(name)

    def unset_option(self, name, key):
        del self.stage(name)["options"][key]
        self._edited(name)

    def save(self, path):
        cache = os.path.relpath(self.pipeline.cache, self.pipeline.root)
        with open(path, "w") as f:
            json.dump({"cache": cache, "inputs": self.pipeline.inputs,
                       "stages": self.pipeline.stages}, f, indent=2)
            f.write("\n")
        self.path = path

    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, until=None):
        """Run the pipeline through the stage `until` (all without) in a background thread; what
        it finishes reaches the window through poll()."""
        def work():
            try:
                self.pipeline.run(until, on_stage=lambda n, r: self.queue.put(("done", n, r)))
            except StageFailed as e:
                self.queue.put(("failed", e.stage, e.log))
            except Exception as e:  # a bad pipeline (a missing file, a reference to nothing)
                self.queue.put(("error", None, f"{type(e).__name__}: {e}"))
        self.message = ""
        self.thread = threading.Thread(target=work, daemon=True)
        self.thread.start()

    def pick_file(self, target):
        """The system's file dialog, the window drawing on meanwhile; the path chosen reaches
        `target` -- "new", the input being added, or an input's name -- through poll()."""
        def work():
            chosen = subprocess.run([sys.executable, "-c", PICK_FILE, self.pipeline.root],
                                    capture_output=True, text=True).stdout.strip()
            if chosen:
                self.queue.put(("picked", target, chosen))
        thread = threading.Thread(target=work, daemon=True)
        thread.start()
        return thread

    def _relative(self, path):
        """`path` relative to the pipeline's folder when inside it, else absolute."""
        rel = os.path.relpath(path, self.pipeline.root)
        return path if rel.startswith("..") else rel

    def _default_name(self, component):
        """A new stage's name until one is typed: <n>_<component>, n its number."""
        n = 1 + len(self.pipeline.stages)
        while f"{n}_{component}" in self.names():
            n += 1
        return f"{n}_{component}"

    def _input(self, name):
        """The mesh the stage's `input` option refers to, for a history's correspondence."""
        if name in self.pipeline.inputs:
            return None
        ref = self.stage(name)["options"].get("input")
        if _is_ref(ref) and "output" in ref and ref["from"] in self.pipeline.results:
            return self.pipeline.results[ref["from"]]["outputs"].get(ref["output"])
        return None

    def poll(self):
        """Take in what the run finished, on the main thread, where polyscope's calls go."""
        while True:
            try:
                kind, name, value = self.queue.get_nowait()
            except queue.Empty:
                return
            if kind == "done":
                self.status[name], self.logs[name] = value["source"], value["log"]
                if self.viewer is not None and self.viewed.get(name) != value["key"]:
                    # A stage run anew is shown alone, if it has a mesh to show (a tracking run
                    # has none); one unchanged keeps its view.
                    for other in self.viewer.stages if value["outputs"] else []:
                        other["shown"] = False
                    self.viewer.set_stage(name, value["outputs"], value["report"],
                                          self._input(name))
                    self.viewed[name] = value["key"]
            elif kind == "picked":
                if name == "new":
                    self.new_input["file"] = self._relative(value)
                elif name in self.pipeline.inputs:
                    self.set_file(name, self._relative(value))
            elif kind == "failed":
                self.status[name], self.logs[name] = "failed", value
                self.log_stage, self.message = name, f"{name} failed: its log is in the log window"
            else:
                self.message = value

    def _log_text(self):
        """The log of the running stage, else of the chosen one, read again when it grows."""
        running = self.pipeline.running
        path = running[1] if running else self.logs.get(self.log_stage)
        if not path or not os.path.exists(path):
            return ""
        size = os.path.getsize(path)
        if (path, size) != (self.log["path"], self.log["size"]):
            with open(path, errors="replace") as f:
                lines = f.read().splitlines()
            self.log = {"path": path, "size": size, "text": "\n".join(lines[-400:])}
        return self.log["text"]

    # The window.

    def _tooltip(self, rule):
        if rule.get("doc") and self.psim.IsItemHovered():
            self.psim.SetTooltip(textwrap.fill(rule["doc"], 90))

    def _json(self, field, label, value):
        """A value as JSON text: (changed, value), the text kept while it does not parse."""
        psim = self.psim
        text = self.text.get(field)
        if text is None or (_parses(text) and json.loads(text) != value):
            text = json.dumps(value)
        changed, text = psim.InputText(label, text)
        self.text[field] = text
        if not _parses(text):
            psim.TextColored(RED, "not JSON")
            return False, value
        return changed, json.loads(text) if changed else value

    def _ref(self, name, label, ref):
        """A reference's widgets: the stage, output, report or option, and which one."""
        psim = self.psim
        above = self.above(name)
        kinds = ["output", "report", "option"]
        kind = next(k for k in kinds if k in ref)
        psim.PushItemWidth(110)
        c_from, k = psim.Combo(f"##from{label}", above.index(ref["from"]), above)
        source = above[k]
        if source in self.pipeline.inputs:  # one output, its mesh: nothing to choose
            psim.PopItemWidth()
            psim.SameLine()
            psim.TextUnformatted(label.split("##")[0])
            return c_from, {"from": source, "output": "mesh"}
        psim.SameLine()
        c_kind, j = psim.Combo(f"##kind{label}", kinds.index(kind), kinds)
        kind = kinds[j]
        psim.SameLine()
        if kind == "option":  # known without a run: the options it sets first, then the rest
            _, required, optional = self.spec(self.stage(source)["component"])
            known = list(self.stage(source)["options"])
            known += [k for k in required + optional if k not in known]
        else:
            result = self.pipeline.results.get(source)
            known = sorted(result["outputs" if kind == "output" else "report"]) if result else []
        first = {"output": "mesh", "report": "", "option": (known or [""])[0]}
        current = ref.get(kind, first[kind])
        if known:
            c_key, i = psim.Combo(label, known.index(current) if current in known else -1, known)
            current = known[i] if c_key else current
        else:  # the stage has not run: its outputs and report keys are not known yet
            c_key, current = psim.InputText(label, current)
        psim.PopItemWidth()
        return c_from or c_kind or c_key, {"from": source, kind: current}

    def _list(self, name, label, value, pointer, rules, unset=None):
        """A list of scalars, an item per row with "x" to remove it; "+" appends a value, "+ ref"
        a reference to the stage just above. `unset` (an optional option): an "x" on the title
        line removes the whole option."""
        psim = self.psim
        psim.TextUnformatted(label.split("##")[0])
        self._tooltip(rules.get(pointer, {}))
        if unset is not None:
            psim.SameLine()
            if psim.SmallButton(f"x##unset{label}"):
                unset()
                return False, value
        changed, out = False, list(value)
        psim.Indent()
        for j, item in enumerate(value):
            c, out[j] = self._value(name, f"##{label}/{j}", item, pointer + "/*", rules)
            changed |= c
            psim.SameLine()
            if psim.SmallButton(f"x##{label}/remove{j}"):
                del out[j]
                changed = True
                break
        if psim.SmallButton(f"+##{label}/append"):
            out.append(_default(rules.get(pointer + "/*", {})))
            changed = True
        above = self.above(name)
        if above:
            psim.SameLine()
            if psim.SmallButton(f"+ ref##{label}/append_ref"):
                out.append({"from": above[-1], "output": "mesh"})
                changed = True
        psim.Unindent()
        return changed, out

    def _value(self, name, label, value, pointer, rules, unset=None):
        """One value's widget, as its spec rule types it, or a reference's; (changed, value).
        `unset`, for an option the stage need not set: an "x" that removes it."""
        psim = self.psim
        rule = rules.get(pointer, {})
        kind = rule.get("type")
        if kind == "list" and isinstance(value, list) \
                and rules.get(pointer + "/*", {}).get("type") in SCALARS:
            return self._list(name, label, value, pointer, rules, unset)
        if _is_ref(value):
            changed, value = self._ref(name, label, value)
        elif kind == "bool" and isinstance(value, bool):
            changed, value = psim.Checkbox(label, value)
        elif kind == "int" and isinstance(value, int) and not isinstance(value, bool):
            changed, value = psim.InputInt(label, value)
        elif kind == "float" and isinstance(value, (int, float)) and not isinstance(value, bool):
            changed, value = psim.InputDouble(label, float(value), format="%.6g")
        elif kind == "string" and value in rule.get("options", []):
            changed, k = psim.Combo(label, rule["options"].index(value), rule["options"])
            value = rule["options"][k]
        elif kind == "string" and isinstance(value, str):
            changed, value = psim.InputText(label, value)
        else:  # objects, lists of objects, and values of another type than their rule's
            changed, value = self._json(f"{name}/{label}", label, value)
        self._tooltip(rule)
        above = self.above(name)
        if kind in SCALARS and above and not pointer.endswith("/*"):  # a list item: see _list
            psim.SameLine()
            if psim.SmallButton(f"{'value' if _is_ref(value) else 'ref'}##toggle{label}"):
                value = _default(rule) if _is_ref(value) else {"from": above[-1], "output": "mesh"}
                changed = True
        if unset is not None:
            psim.SameLine()
            if psim.SmallButton(f"x##unset{label}"):
                unset()
                return False, value
        return changed, value

    def _options(self, stage):
        psim = self.psim
        name = stage["name"]
        rules, required, optional = self.spec(stage["component"])
        for key in list(stage["options"]):
            unset = None if key in required else lambda key=key: self.unset_option(name, key)
            c, value = self._value(name, f"{key}##{name}/{key}", stage["options"][key], f"/{key}",
                                   rules, unset)
            if c:
                self.set_option(name, key, value)
        unset = [k for k in required + optional if k not in stage["options"]]
        if not unset or not _viewer._node(psim, f"other options ({len(unset)})##other{name}"):
            return
        _, self.filters[name] = psim.InputText(f"filter##filter{name}", self.filters.get(name, ""))
        keys = [k for k in unset if self.filters[name].lower() in k.lower()]
        special = [k for k in keys if k.startswith(("EXPERIMENTAL_", "DEBUG_"))]
        for group, group_keys in [(None, [k for k in keys if k not in special]),
                                  ("EXPERIMENTAL_ and DEBUG_", special)]:
            if not group_keys or group and not _viewer._node(
                    psim, f"{group} ({len(group_keys)})##special{name}"):
                continue
            for key in group_keys:
                rule = rules.get(f"/{key}", {})
                if psim.SmallButton(f"+##add{name}/{key}"):
                    self.set_option(name, key, _default(rule))
                psim.SameLine()
                psim.TextUnformatted(key)
                self._tooltip(rule)
                psim.SameLine()
                default = json.dumps(rule["default"]) if "default" in rule else "no default"
                psim.TextDisabled(default if len(default) <= 40 else default[:37] + "...")
            if group:
                psim.TreePop()
        psim.TreePop()

    def _remove(self, name):
        try:
            self.remove(name)
        except ValueError as e:
            self.message = str(e)

    def _input_row(self, name, file):
        """An input as a row of the inputs table: its name (editable; every reference to it
        follows), its path with "..." for the system's file dialog, "x" to remove it."""
        psim = self.psim
        psim.TableNextRow()
        psim.TableSetColumnIndex(0)
        psim.PushItemWidth(-1)
        _, self.renaming[name] = psim.InputText(f"##name{name}", self.renaming.get(name, name))
        psim.PopItemWidth()
        if psim.IsItemDeactivatedAfterEdit():  # Enter, or a click elsewhere
            try:
                self.rename(name, self.renaming.pop(name))
            except ValueError as e:
                self.message = str(e)
        psim.TableSetColumnIndex(1)
        missing = not os.path.exists(os.path.join(self.pipeline.root, file))
        psim.PushItemWidth(-100 if missing else -30)
        c, file = psim.InputText(f"##file{name}", file)
        psim.PopItemWidth()
        if c:
            self.set_file(name, file)
        psim.SameLine()
        if psim.SmallButton(f"...##pick{name}"):
            self.pick_file(name)
        if missing:
            psim.SameLine()
            psim.TextColored(RED, "missing")
        psim.TableSetColumnIndex(2)
        if psim.SmallButton(f"x##remove{name}"):
            self._remove(name)

    def _new_input_row(self):
        """The inputs table's last row: an input to add, named after its file until a name is
        typed."""
        psim = self.psim
        new = self.new_input
        default = os.path.splitext(os.path.basename(new["file"]))[0]
        if new["name"] in ("", new.get("default")):
            new["name"] = default
        new["default"] = default
        psim.TableNextRow()
        psim.TableSetColumnIndex(0)
        psim.PushItemWidth(-1)
        _, new["name"] = psim.InputText("##newname", new["name"])
        psim.PopItemWidth()
        psim.TableSetColumnIndex(1)
        psim.PushItemWidth(-30)
        _, new["file"] = psim.InputText("##newfile", new["file"])
        psim.PopItemWidth()
        psim.SameLine()
        if psim.SmallButton("...##picknew"):
            self.pick_file("new")
        psim.TableSetColumnIndex(2)
        if psim.SmallButton("+##addinput"):
            try:
                self.add_input(new["name"], new["file"])
                self.new_input = {"name": "", "file": ""}
            except ValueError as e:
                self.message = str(e)

    def _stage(self, i, stage):
        psim = self.psim
        name = stage["name"]
        running = self.pipeline.running
        status = "running" if running and running[0] == name else self.status.get(name, "not run")
        if not _viewer._node(psim, f"{i + 1}. {name}: {stage['component']} [{status}]"
                                   f"###stage{name}"):
            return
        self._options(stage)
        if psim.Button(f"run##run{name}"):
            self.start(name)
        if psim.IsItemHovered():
            psim.SetTooltip("Run this stage. The stages above it run only if they changed; the\n"
                            "others come from memory or the cache. The stages below wait.")
        psim.SameLine()
        if psim.Button(f"log##log{name}"):
            self.log_stage = name
        psim.SameLine()
        if psim.Button(f"remove##remove{name}"):
            self._remove(name)
        psim.TreePop()

    def _add(self):
        """The row adding a stage: its component, its name (<n>_<component> until typed over)."""
        psim = self.psim
        kinds = wm.components()
        _, self.new["kind"] = psim.Combo("new stage", self.new["kind"], kinds)
        default = self._default_name(kinds[self.new["kind"]])
        if self.new["name"] in ("", self.new.get("default")):  # not typed over: follow the kind
            self.new["name"] = default
        self.new["default"] = default
        _, self.new["name"] = psim.InputText("name##new", self.new["name"])
        if psim.Button("add stage"):
            try:
                self.add_stage(self.new["name"], kinds[self.new["kind"]])
            except ValueError as e:
                self.message = str(e)

    def panel(self):
        """Every frame: what the run finished, then the windows. polyscope's own on the left,
        folded at start; on the right "pipeline" and "view" (the stages' panel) below it, which
        takes the height the pipeline leaves; "log" along the bottom between the two columns,
        folded at start and unrolling upwards."""
        psim = self.psim
        self.poll()
        busy = self.running()
        _viewer.polyscope_panels(self.ps, psim)
        width, height = psim.GetIO().DisplaySize
        left = 2 * MARGIN + LEFT * self.ps.get_ui_scale()
        # The right column keeps to the right edge at the width the pipeline window is dragged
        # to (by its left edge), from RIGHT at start.
        psim.SetNextWindowPos((width - self.column - MARGIN, MARGIN), psim.ImGuiCond_Always)
        psim.SetNextWindowSize((self.column, 0.6 * height), psim.ImGuiCond_Once)
        psim.SetNextWindowSizeConstraints((300, 60), (max(width - left - MARGIN, 300),
                                                      height - 2 * MARGIN))
        psim.Begin("pipeline", True, psim.ImGuiWindowFlags_HorizontalScrollbar)
        _, self.path = psim.InputText("pipeline file", self.path)
        if psim.Button("save"):
            try:
                self.save(self.path)
                self.message = f"saved {self.path}"
            except OSError as e:
                self.message = str(e)
        psim.SameLine()
        if psim.Button("run all") and not busy:
            self.start()
        if busy:
            psim.SameLine()
            running = self.pipeline.running
            psim.TextUnformatted(f"running {running[0]}" if running else "running")
        if self.message:
            psim.TextWrapped(self.message)
        psim.Separator()
        psim.BeginDisabled(busy)
        if psim.BeginTable("inputs", 3):
            psim.TableSetupColumn("input name", psim.ImGuiTableColumnFlags_WidthFixed, 90)
            psim.TableSetupColumn("file path", psim.ImGuiTableColumnFlags_WidthStretch)
            psim.TableSetupColumn("", psim.ImGuiTableColumnFlags_WidthFixed, 20)
            psim.TableHeadersRow()
            for name, file in list(self.pipeline.inputs.items()):
                self._input_row(name, file)
            self._new_input_row()
            psim.EndTable()
        psim.Separator()
        for i, stage in enumerate(list(self.pipeline.stages)):
            self._stage(i, stage)
        psim.Separator()
        self._add()
        psim.EndDisabled()
        pipeline_height = psim.GetWindowHeight()  # its title bar only, when folded
        self.column = psim.GetWindowWidth()
        psim.End()

        right = width - self.column - MARGIN
        top = 2 * MARGIN + pipeline_height
        psim.SetNextWindowPos((right, top), psim.ImGuiCond_Always)
        psim.SetNextWindowSize((self.column, max(height - top - MARGIN, 60)),
                               psim.ImGuiCond_Always)
        psim.Begin("view", True, psim.ImGuiWindowFlags_HorizontalScrollbar)
        self.viewer.panel()
        psim.End()

        # The log spans the gap between the columns; its height is the one thing to drag.
        span = max(right - MARGIN - left, 200)
        psim.SetNextWindowPos((left, height - MARGIN), psim.ImGuiCond_Always, (0.0, 1.0))
        psim.SetNextWindowSize((span, 0.35 * height), psim.ImGuiCond_Once)
        psim.SetNextWindowSizeConstraints((span, 60), (span, height - 2 * MARGIN))
        psim.SetNextWindowCollapsed(True, psim.ImGuiCond_Once)
        running = self.pipeline.running
        shown = running[0] if running else self.log_stage
        # The title says whose log it is: folded, the title bar is all there is.
        title = f"log: {shown}{' (running)' if running else ''}" if shown else "log"
        expanded, _ = psim.Begin(f"{title}###log", True)
        if expanded:
            if not shown:
                psim.TextUnformatted("no log chosen: a stage's \"log\" button")
            psim.BeginChild("log text")
            psim.TextUnformatted(self._log_text())
            if running:
                psim.SetScrollHereY(1.0)
            psim.EndChild()
        psim.End()


def run_window(builder, screenshots=None):
    """Open `builder`'s window; `screenshots` "mock" (or WM_VIEWER_SCREENSHOTS) draws three frames
    without one, as a self-test."""
    import polyscope as ps
    import polyscope.imgui as psim

    mode = _viewer.start(ps, screenshots)
    ps.set_open_imgui_window_for_user_callback(False)  # the builder places its own windows
    builder.ps, builder.psim = ps, psim
    if builder.viewer is None:
        builder.viewer = _viewer.Viewer(ps, psim)
    ps.set_user_callback(builder.panel)
    if mode == "mock":
        ps.show(3)
    else:
        ps.show()


def main(argv):
    if argv and os.path.exists(argv[0]):
        builder = Builder(Pipeline.load(argv[0]), argv[0])
    else:
        builder = Builder(Pipeline([], os.getcwd()), argv[0] if argv else None)
    run_window(builder)


if __name__ == "__main__":
    main(sys.argv[1:])
