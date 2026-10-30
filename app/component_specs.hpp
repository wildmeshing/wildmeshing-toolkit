#pragma once

// What the components' JSON specs say about themselves, for wmtk_app (--list, --options, and the
// warning for options a spec does not have) and for the Python module's docstrings. The specs
// themselves come from components_spec_map.hpp, which add_component generates.

#include <nlohmann/json.hpp>

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace wmtk::component_specs {

using SpecMap = std::map<std::string, std::function<const nlohmann::json&()>>;

inline const std::string reference_url =
    "https://wildmeshing.github.io/wildmeshing-toolkit/reference/";

inline const nlohmann::json& root_rule(const nlohmann::json& spec)
{
    for (const auto& rule : spec) {
        if (rule.value("pointer", "") == "/") return rule;
    }
    static const nlohmann::json none = nlohmann::json::object();
    return none;
}

inline std::string first_sentence(const std::string& text)
{
    for (size_t i = 0; i + 1 < text.size(); ++i) {
        if ((text[i] == '.' || text[i] == '!' || text[i] == '?') && text[i + 1] == ' ') {
            return text.substr(0, i + 1);
        }
    }
    return text;
}

// The top-level options a spec has, in its order.
inline std::vector<std::string> option_names(const nlohmann::json& spec)
{
    std::vector<std::string> names;
    for (const auto& rule : spec) {
        const std::string pointer = rule.value("pointer", "");
        if (pointer.size() > 1 && pointer.find('/', 1) == std::string::npos &&
            std::find(names.begin(), names.end(), pointer.substr(1)) == names.end()) {
            names.push_back(pointer.substr(1));
        }
    }
    return names;
}

// The keys of options that the spec does not have. A component ignores them, except
// topological_offset, which refuses them. input_dir (injected by wmtk_app and the bindings) and
// $schema (an editor's, removed before the component runs) are not reported.
inline std::vector<std::string> unknown_options(
    const nlohmann::json& spec,
    const nlohmann::json& options)
{
    std::vector<std::string> unknown;
    if (!options.is_object()) return unknown;
    const auto names = option_names(spec);
    const std::set<std::string> known(names.begin(), names.end());
    for (const auto& [key, value] : options.items()) {
        if (!known.count(key) && key != "input_dir" && key != "$schema") unknown.push_back(key);
    }
    return unknown;
}

// One line per option: name, type, default, and the first sentence of its doc.
inline std::string format_options(const nlohmann::json& spec, const std::string& indent = "  ")
{
    const auto& root = root_rule(spec);
    std::set<std::string> required;
    for (const auto& r : root.value("required", nlohmann::json::array())) {
        required.insert(r.get<std::string>());
    }
    std::string out;
    for (const auto& name : option_names(spec)) {
        for (const auto& rule : spec) {
            if (rule.value("pointer", "") != "/" + name) continue;
            out += indent + name + " (" + rule.value("type", "");
            if (required.count(name)) {
                out += ", required";
            } else if (rule.contains("default")) {
                out += ", default " + rule["default"].dump();
            }
            out += ")";
            const std::string doc = first_sentence(rule.value("doc", ""));
            if (!doc.empty()) out += ": " + doc;
            out += "\n";
            break;
        }
    }
    return out;
}

// What `help(wildmeshing.<name>)` and `wmtk_app --options <name>` show.
inline std::string describe(const std::string& name, const nlohmann::json& spec)
{
    std::string out = root_rule(spec).value("doc", "");
    if (!out.empty()) out += "\n\n";
    out += "Options (every one, with its full description: " + reference_url + name + "/):\n";
    out += format_options(spec);
    return out;
}

} // namespace wmtk::component_specs
