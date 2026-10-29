// wm.Tracking and wm.CurveTracking: the tracking component's maps between an application's input
// and its output, built from a history -- the bytes a memory call returns as report["history"],
// or the file the file route writes -- and mapping NumPy arrays of points either way.

#include "mesh_conversion.hpp"

#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11_json/pybind11_json.hpp>

#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/utils/Logger.hpp>

#include <array>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
namespace {

using components::tracking::CurveMapOptions;
using components::tracking::CurveMapWarnings;
using components::tracking::CurveTracking;
using components::tracking::InputCurvePoint;
using components::tracking::InputPoint;
using components::tracking::kNoId;
using components::tracking::MapWarnings;
using components::tracking::OutputCurvePoint;
using components::tracking::OutputPoint;
using components::tracking::Scaffold;
using components::tracking::Smoothing;
using components::tracking::SurfaceMapOptions;
using components::tracking::Tracking;

std::vector<uint8_t> bytes_of(const py::bytes& history)
{
    const std::string data = history;
    return {data.begin(), data.end()};
}

Smoothing smoothing_of(const std::string& smoothing)
{
    if (smoothing == "flatten") return Smoothing::flatten;
    if (smoothing == "barycentric") return Smoothing::barycentric;
    throw py::value_error("smoothing is 'flatten' or 'barycentric', not '" + smoothing + "'");
}

SurfaceMapOptions surface_options(const std::string& smoothing, const std::string& scaffold)
{
    SurfaceMapOptions options;
    options.smoothing = smoothing_of(smoothing);
    if (scaffold == "ring") {
        options.scaffold = Scaffold::ring;
    } else if (scaffold != "box") {
        throw py::value_error("scaffold is 'box' or 'ring', not '" + scaffold + "'");
    }
    return options;
}

/// A history of faces of `n` corners (history::face_size) is to be mapped by the class for
/// `corners`: the other class maps the other kind.
void require_kind(const int n, const int corners)
{
    if (n == corners) return;
    throw py::value_error(
        n == 2 ? "the history is of curves (triwild's): map it with CurveTracking"
               : "the history is of a surface: map it with Tracking");
}

/**
 * @brief A map, built by `build` with the GIL released: building may take long, and logs through
 * the process-wide logger a memory call swaps for its own, so it holds the lock component calls
 * hold (wmtk::detail::ComponentLoggerScope) and runs between them. A damaged history throws
 * before anything is built: RuntimeError in Python.
 */
template <typename Map, typename Build>
std::unique_ptr<Map> built(Build&& build)
{
    py::gil_scoped_release release;
    wmtk::detail::ComponentLoggerScope lock(false);
    return build();
}

uint64_t id_of(const int64_t id)
{
    return id < 0 ? kNoId : uint64_t(id);
}
int64_t id_out(const uint64_t id)
{
    return id == kNoId || id > uint64_t(std::numeric_limits<int64_t>::max()) ? -1 : int64_t(id);
}

/**
 * @brief The ids of n points: shape (n,) for `columns` 0, (n, columns) otherwise, of an integer
 * dtype, -1 for none. `naming` says why that shape, in the error for another.
 */
DataMatrix<int64_t> read_ids(py::handle value, const int columns, const std::string& naming)
{
    if (!py::isinstance<py::array>(value)) {
        throw py::type_error("ids must be a NumPy array of integers");
    }
    const auto a = py::reinterpret_borrow<py::array>(value);
    if (a.dtype().kind() != 'i' && a.dtype().kind() != 'u') {
        throw py::type_error(
            "ids must have an integer dtype, not " + std::string(py::str(a.dtype())));
    }
    const bool shaped = columns == 0 ? a.ndim() == 1 : a.ndim() == 2 && a.shape(1) == columns;
    if (!shaped) {
        throw py::value_error(
            "ids must be of shape " +
            (columns == 0 ? std::string("(n,)") : "(n, " + std::to_string(columns) + ")") + ": " +
            naming);
    }
    DataMatrix<int64_t> ids = integers(a, true);
    for (Eigen::Index i = 0; i < ids.size(); ++i) {
        if (ids.data()[i] < -1) {
            throw py::value_error(
                "ids holds " + std::to_string(ids.data()[i]) +
                ": an id is not negative, or -1 for none");
        }
    }
    return ids;
}

/// The barycentric coordinates of n points, (n, k) of a floating dtype.
DataMatrix<double> read_bary(py::handle value, const Eigen::Index n, const int k)
{
    if (!py::isinstance<py::array>(value)) {
        throw py::type_error("bary must be a NumPy array of floats");
    }
    const auto a = py::reinterpret_borrow<py::array>(value);
    if (a.dtype().kind() != 'f') {
        throw py::type_error(
            "bary must have a floating dtype, not " + std::string(py::str(a.dtype())));
    }
    if (a.ndim() != 2 || a.shape(0) != n || a.shape(1) != k) {
        throw py::value_error(
            "bary must be of shape (n, " + std::to_string(k) + "), one row per id: here (" +
            std::to_string(n) + ", " + std::to_string(k) + ")");
    }
    return matrix<double>(a);
}

/// Ids of n points as NumPy returns them: (n,) for one a point, (n, k) for k; -1 for none.
template <size_t K>
py::array ids_array(const std::vector<std::array<uint64_t, K>>& ids, const bool one)
{
    const py::ssize_t n = py::ssize_t(ids.size());
    py::array_t<int64_t> out(
        one ? std::vector<py::ssize_t>{n} : std::vector<py::ssize_t>{n, py::ssize_t(K)});
    int64_t* data = out.mutable_data();
    for (size_t i = 0; i < ids.size(); ++i) {
        if (one) {
            data[i] = id_out(ids[i][0]);
            continue;
        }
        for (size_t k = 0; k < K; ++k) data[K * i + k] = id_out(ids[i][k]);
    }
    return out;
}

/// The coordinates of mapped points, NaN for one with no image.
template <typename Point>
py::array bary_array(const std::vector<Point>& points, const std::vector<uint8_t>& found)
{
    constexpr int K = decltype(Point::bary)::RowsAtCompileTime;
    py::array_t<double> out(std::vector<py::ssize_t>{py::ssize_t(points.size()), py::ssize_t(K)});
    double* data = out.mutable_data();
    for (size_t i = 0; i < points.size(); ++i) {
        for (int k = 0; k < K; ++k) {
            data[K * i + k] =
                found[i] ? points[i].bary[k] : std::numeric_limits<double>::quiet_NaN();
        }
    }
    return out;
}

py::tuple result(py::array ids, py::array bary, const bool warnings, const nlohmann::json& report)
{
    if (!warnings) return py::make_tuple(ids, bary);
    return py::make_tuple(ids, bary, pyjson::from_json(report));
}

/// Faces of a side as NumPy holds them: (m, K) Int64, -1 for an id the side does not name.
template <size_t K>
py::array faces_array(const std::vector<std::array<uint64_t, K>>& faces)
{
    return ids_array(faces, false);
}

constexpr const char* kIds =
    "ids: int64 (n,), a face (segment) per point, or (n, 3) -- (n, 2) on curves -- vertices "
    "where the side names points by the vertices of a mesh (output_by_vertices, "
    "input_by_vertices); -1 for none. bary: float64 (n, 3) -- (n, 2) on curves -- barycentric "
    "coordinates in the order of the face's corners, or of the vertices given. Returns (ids, "
    "bary) on the other side, in its naming, -1 and NaN for a point with no image; with "
    "warnings=True, (ids, bary, warnings), what this call found where the map is not a "
    "bijection. The GIL is released while mapping: several threads may map through one object.";

py::tuple surface_to_output(const Tracking& t, py::handle ids, py::handle bary, const bool warnings)
{
    const bool by_vertices = t.input_by_vertices();
    const DataMatrix<int64_t> I = read_ids(
        ids,
        by_vertices ? 3 : 0,
        by_vertices ? "this history names input points by three vertices of the mesh it starts "
                      "from (input_by_vertices)"
                    : "this history names input points by triangles of its input files");
    const DataMatrix<double> B = read_bary(bary, I.rows(), 3);
    std::vector<InputPoint> pts(size_t(I.rows()));
    for (Eigen::Index i = 0; i < I.rows(); ++i) {
        if (by_vertices) {
            for (int k = 0; k < 3; ++k) pts[i].vertices[k] = id_of(I(i, k));
        } else {
            pts[i].face = id_of(I(i, 0));
        }
        pts[i].bary = B.row(i).transpose();
    }
    std::vector<OutputPoint> out;
    MapWarnings w;
    {
        py::gil_scoped_release release;
        out = t.to_output(pts, &w);
    }
    const bool out_vertices = t.output_by_vertices();
    std::vector<std::array<uint64_t, 3>> named(out.size());
    std::vector<uint8_t> found(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        named[i] = out_vertices ? out[i].vertices : std::array<uint64_t, 3>{{out[i].face}};
        found[i] = named[i][0] != kNoId;
        if (!found[i]) named[i] = {{kNoId, kNoId, kNoId}};
    }
    return result(
        ids_array(named, !out_vertices),
        bary_array(out, found),
        warnings,
        components::tracking::warnings_report(w));
}

py::tuple surface_to_input(const Tracking& t, py::handle ids, py::handle bary, const bool warnings)
{
    const bool by_vertices = t.output_by_vertices();
    const DataMatrix<int64_t> I = read_ids(
        ids,
        by_vertices ? 3 : 0,
        by_vertices ? "this history names output points by three vertices of the output tet "
                      "mesh (output_by_vertices)"
                    : "this history names output points by triangles of its output");
    const DataMatrix<double> B = read_bary(bary, I.rows(), 3);
    std::vector<OutputPoint> pts(size_t(I.rows()));
    for (Eigen::Index i = 0; i < I.rows(); ++i) {
        if (by_vertices) {
            for (int k = 0; k < 3; ++k) pts[i].vertices[k] = id_of(I(i, k));
        } else {
            pts[i].face = id_of(I(i, 0));
        }
        pts[i].bary = B.row(i).transpose();
    }
    std::vector<InputPoint> out;
    MapWarnings w;
    {
        py::gil_scoped_release release;
        out = t.to_input(pts, &w);
    }
    const bool in_vertices = t.input_by_vertices();
    std::vector<std::array<uint64_t, 3>> named(out.size());
    std::vector<uint8_t> found(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        named[i] = in_vertices ? out[i].vertices : std::array<uint64_t, 3>{{out[i].face}};
        found[i] = named[i][0] != kNoId;
        if (!found[i]) named[i] = {{kNoId, kNoId, kNoId}};
    }
    return result(
        ids_array(named, !in_vertices),
        bary_array(out, found),
        warnings,
        components::tracking::warnings_report(w));
}

py::tuple
curve_to_output(const CurveTracking& t, py::handle ids, py::handle bary, const bool warnings)
{
    const DataMatrix<int64_t> I =
        read_ids(ids, 0, "input points are named by segments of the input curve files");
    const DataMatrix<double> B = read_bary(bary, I.rows(), 2);
    std::vector<InputCurvePoint> pts(size_t(I.rows()));
    for (Eigen::Index i = 0; i < I.rows(); ++i) {
        pts[i].segment = id_of(I(i, 0));
        pts[i].bary = B.row(i).transpose();
    }
    std::vector<OutputCurvePoint> out;
    CurveMapWarnings w;
    {
        py::gil_scoped_release release;
        out = t.to_output(pts, &w);
    }
    const bool out_vertices = t.output_by_vertices();
    std::vector<std::array<uint64_t, 2>> named(out.size());
    std::vector<uint8_t> found(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        named[i] = out_vertices ? out[i].vertices : std::array<uint64_t, 2>{{out[i].segment}};
        found[i] = named[i][0] != kNoId;
        if (!found[i]) named[i] = {{kNoId, kNoId}};
    }
    return result(
        ids_array(named, !out_vertices),
        bary_array(out, found),
        warnings,
        components::tracking::warnings_report(w));
}

py::tuple
curve_to_input(const CurveTracking& t, py::handle ids, py::handle bary, const bool warnings)
{
    const bool by_vertices = t.output_by_vertices();
    const DataMatrix<int64_t> I = read_ids(
        ids,
        by_vertices ? 2 : 0,
        by_vertices ? "this history names output points by two vertices of the output mesh "
                      "(output_by_vertices)"
                    : "this history names output points by segments of its output");
    const DataMatrix<double> B = read_bary(bary, I.rows(), 2);
    std::vector<OutputCurvePoint> pts(size_t(I.rows()));
    for (Eigen::Index i = 0; i < I.rows(); ++i) {
        if (by_vertices) {
            for (int k = 0; k < 2; ++k) pts[i].vertices[k] = id_of(I(i, k));
        } else {
            pts[i].segment = id_of(I(i, 0));
        }
        pts[i].bary = B.row(i).transpose();
    }
    std::vector<InputCurvePoint> out;
    CurveMapWarnings w;
    {
        py::gil_scoped_release release;
        out = t.to_input(pts, &w);
    }
    std::vector<std::array<uint64_t, 1>> named(out.size());
    std::vector<uint8_t> found(out.size());
    for (size_t i = 0; i < out.size(); ++i) {
        named[i] = {{out[i].segment}};
        found[i] = out[i].segment != kNoId;
    }
    return result(
        ids_array(named, true),
        bary_array(out, found),
        warnings,
        components::tracking::warnings_report(w));
}

} // namespace

void bind_tracking(py::module_& module)
{
    py::class_<Tracking>(
        module,
        "Tracking",
        "The map between a surface application's input and its output, both ways, built from the "
        "history it recorded (its history_output): the bytes a memory call returns as "
        "report['history'], or Tracking.load(path) of the file the file route writes. Points on "
        "the input are named by triangles of the input files, or, for a history that starts at a "
        "tet mesh (input_by_vertices), by three of its vertices; points on the output by "
        "triangles of a surface output, or by three vertices of a tet output "
        "(output_by_vertices). Mapping never changes the object.")
        .def(
            py::init([](const py::bytes& history,
                        const std::string& smoothing,
                        const std::string& scaffold) {
                const std::vector<uint8_t> bytes = bytes_of(history);
                const SurfaceMapOptions options = surface_options(smoothing, scaffold);
                require_kind(history::face_size(bytes), 3);
                return built<Tracking>([&] {
                    return std::make_unique<Tracking>(
                        history::SurfaceHistory::from_bytes(bytes),
                        options);
                });
            }),
            "history"_a,
            "smoothing"_a = "flatten",
            "scaffold"_a = "box",
            "Build the map from the bytes of a surface history. smoothing ('flatten' or "
            "'barycentric') and scaffold ('box' or 'ring') as the tracking component's options.")
        .def_static(
            "load",
            [](const std::filesystem::path& path,
               const std::string& smoothing,
               const std::string& scaffold) {
                const SurfaceMapOptions options = surface_options(smoothing, scaffold);
                require_kind(history::face_size(path.string()), 3);
                return built<Tracking>([&] {
                    return std::make_unique<Tracking>(
                        history::SurfaceHistory::load(path.string()),
                        options);
                });
            },
            "path"_a,
            "smoothing"_a = "flatten",
            "scaffold"_a = "box",
            "Build the map from a history file, as from its bytes (wm.read_history).")
        .def(
            "to_output",
            &surface_to_output,
            "ids"_a,
            "bary"_a,
            "warnings"_a = false,
            (std::string("Points on the input, to the output. ") + kIds).c_str())
        .def(
            "to_input",
            &surface_to_input,
            "ids"_a,
            "bary"_a,
            "warnings"_a = false,
            (std::string(
                 "Points on the output, to the input: the inverse map. Vertices may be "
                 "given in any order, the coordinates following them. ") +
             kIds)
                .c_str())
        .def_property_readonly(
            "output_by_vertices",
            &Tracking::output_by_vertices,
            "Whether output points are named by three vertices of a tet mesh, not by triangles.")
        .def_property_readonly(
            "input_by_vertices",
            &Tracking::input_by_vertices,
            "Whether input points are named by three vertices of the tet mesh the history starts "
            "from, not by triangles of input files.")
        .def_property_readonly(
            "input_faces",
            [](const Tracking& t) { return faces_array(t.input_faces()); },
            "int64 (m, 3): the triangles of the input files, in order, as the history holds them.")
        .def_property_readonly(
            "input_triangles",
            [](const Tracking& t) { return faces_array(t.input_triangles()); },
            "int64 (m, 3): the input's tracked faces, in the ids input points use.")
        .def_property_readonly(
            "output_triangles",
            [](const Tracking& t) { return faces_array(t.output_triangles()); },
            "int64 (k, 3): the output's tracked faces, in the ids output points use.")
        .def_property_readonly(
            "build_warnings",
            [](const Tracking& t) {
                return pyjson::from_json(
                    components::tracking::warnings_report(t.map().build_warnings()));
            },
            "What building the map found where it is not a bijection, by kind.");

    py::class_<CurveTracking>(
        module,
        "CurveTracking",
        "The map between a curve application's input curves and its output's (triwild), both "
        "ways, built from the history it recorded: the bytes of report['history'], or "
        "CurveTracking.load(path). Points on the input are named by segments of the input curve "
        "files -- the segments of all of them, in order, a polyline's consecutive vertex pairs "
        "-- and points on the output by the two vertices of a tracked edge of the output mesh "
        "(output_by_vertices). Mapping never changes the object.")
        .def(
            py::init([](const py::bytes& history, const std::string& smoothing) {
                const std::vector<uint8_t> bytes = bytes_of(history);
                CurveMapOptions options;
                options.smoothing = smoothing_of(smoothing);
                require_kind(history::face_size(bytes), 2);
                return built<CurveTracking>([&] {
                    return std::make_unique<CurveTracking>(
                        history::CurveHistory::from_bytes(bytes),
                        options);
                });
            }),
            "history"_a,
            "smoothing"_a = "flatten",
            "Build the map from the bytes of a curve history. smoothing: 'flatten' or "
            "'barycentric'.")
        .def_static(
            "load",
            [](const std::filesystem::path& path, const std::string& smoothing) {
                CurveMapOptions options;
                options.smoothing = smoothing_of(smoothing);
                require_kind(history::face_size(path.string()), 2);
                return built<CurveTracking>([&] {
                    return std::make_unique<CurveTracking>(
                        history::CurveHistory::load(path.string()),
                        options);
                });
            },
            "path"_a,
            "smoothing"_a = "flatten",
            "Build the map from a curve history file, as from its bytes.")
        .def(
            "to_output",
            &curve_to_output,
            "ids"_a,
            "bary"_a,
            "warnings"_a = false,
            (std::string("Points on the input curves, to the output. ") + kIds).c_str())
        .def(
            "to_input",
            &curve_to_input,
            "ids"_a,
            "bary"_a,
            "warnings"_a = false,
            (std::string(
                 "Points on the output curves, to the input: the inverse map. The two "
                 "vertices may be given in either order, the coordinates following "
                 "them. ") +
             kIds)
                .c_str())
        .def_property_readonly(
            "output_by_vertices",
            &CurveTracking::output_by_vertices,
            "Whether output points are named by two vertices of a mesh, not by segments.")
        .def_property_readonly(
            "input_segments",
            [](const CurveTracking& t) { return faces_array(t.input_segments()); },
            "int64 (m, 2): the segments of the input curve files, in order.")
        .def_property_readonly(
            "output_segments",
            [](const CurveTracking& t) { return faces_array(t.output_segments()); },
            "int64 (k, 2): the output's tracked edges, in the ids output points use.")
        .def(
            "feature_points",
            [](const CurveTracking& t) {
                std::vector<std::array<uint64_t, 2>> pairs;
                for (const auto& f : t.feature_points()) {
                    pairs.push_back({{f.input_vertex, f.output_vertex}});
                }
                return faces_array(pairs);
            },
            "int64 (p, 2): the input's feature points -- curve ends, junctions, input points -- "
            "each a vertex of the input files (the curve files', in order, then the point "
            "files') and the output vertex it ended at, -1 for none.")
        .def_property_readonly(
            "build_warnings",
            [](const CurveTracking& t) {
                return pyjson::from_json(
                    components::tracking::warnings_report(t.map().build_warnings()));
            },
            "What building the map found where it is not a bijection, by kind.");
}

} // namespace wmtk::python
