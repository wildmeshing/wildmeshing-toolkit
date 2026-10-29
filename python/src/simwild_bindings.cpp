// simwild's tag expressions, for Python: the grammar its selections use, parsed by the component's
// own parser.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <wmtk/components/simwild/expression_parser/Parser.hpp>

#include <map>
#include <set>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
namespace {

// Parsed tag expression evaluable from Python over sets of group NAMES.
// Single source of truth for the grammar shared with the Python-side ops
// (wildmeshing.polyfem_ops.mesh_core delegates here instead of mirroring the
// parser).
class PyExpression
{
public:
    PyExpression(const std::string& expr, const std::vector<std::string>& known_names)
    {
        int64_t i = 0;
        for (const auto& n : known_names) {
            m_name_to_id.emplace(n, i++);
        }
        m_expr = wmtk::components::simwild::expression_parser::parse(expr, m_name_to_id);
    }

    bool eval(const std::set<std::string>& tags) const
    {
        // Names outside known_names still count toward the tag set (they can
        // never satisfy a name atom, but `_` = emptiness must see them) —
        // mirrors SimWildMesh::string_set_to_cell_tag assigning fresh ids.
        wmtk::components::simwild::CellTag ct;
        int64_t fresh = static_cast<int64_t>(m_name_to_id.size());
        for (const auto& t : tags) {
            const auto it = m_name_to_id.find(t);
            ct.insert(it != m_name_to_id.end() ? it->second : fresh++);
        }
        return m_expr->eval(ct);
    }

    bool has_unknown_names() const
    {
        // unknown identifiers parse to TagExpr(-1)
        return m_expr->tags_involved().count(-1) > 0;
    }

private:
    std::map<std::string, int64_t> m_name_to_id;
    wmtk::components::simwild::expression_parser::ExpressionPtr m_expr;
};

} // namespace

void bind_simwild(py::module_& module)
{
    py::class_<PyExpression>(module, "Expression")
        .def(
            py::init<const std::string&, const std::vector<std::string>&>(),
            "expr"_a,
            "known_names"_a,
            "Parse a tag expression (names, &, |, !, parentheses, _) "
            "against the given known names.")
        .def("eval", &PyExpression::eval, "tags"_a, "Evaluate against a set of tag names.")
        .def(
            "has_unknown_names",
            &PyExpression::has_unknown_names,
            "True if the expression references names outside known_names.");
}

} // namespace wmtk::python
