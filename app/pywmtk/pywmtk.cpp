#include <pybind11/pybind11.h>

#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>
#include <pybind11_json/pybind11_json.hpp>

#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

#include <spdlog/async.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

// components
#include "../components_include.hpp"

// std::map<std::string, std::function<void(nlohmann::json)>> components_map;
//// include auto-generated map
//#include "../components_map.hpp"

namespace py = pybind11;
using namespace pybind11::literals;


void wmtk_wrapper(const py::dict& obj)
{
    const nlohmann::json j = obj;

    std::cout << "Hello" << std::endl;
    std::cerr << "Hello ERR" << std::endl;

    using namespace wmtk;
    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include "../components_map.hpp"

    std::cout << "Before set logger" << std::endl;
    std::cout << "CHECK 3" << std::endl;
    {
        auto l = spdlog::stdout_color_mt("wmtk");
        std::cout << "Before set logger 2" << std::endl;
        // auto l = spdlog::stdout_logger_mt("wmtk");
        // auto null_sink = std::make_shared<spdlog::sinks::null_sink_mt>();
        // auto l = std::make_shared<spdlog::logger>("wmtk-pywmtk", null_sink);
        //  auto l = spdlog::null_logger_mt("wmtk-pywmtk");
        // l->set_level(spdlog::level::info);
        set_logger(l);
    }
    std::cout << "After set logger" << std::endl;

    // make sure input file contains the application name
    if (!j.contains("application")) {
        logger().info("App not found");
        std::cerr << "Application not found" << std::endl;
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Applictaion {} unknown", app_str);
    }

    logger().info("Input type: {}", j["input"].type_name());
    // logger().info("Input: {}", j["input"]);

    // execute
    components_map[app_str](j);
}

PYBIND11_MODULE(pywmtk, m, py::mod_gil_not_used())
{
    m.doc() = "Python bindings for the Wildmeshing-Toolkit"; // optional module docstring
    // m.def("get_metrics", &meme::get_metrics, "Get mesh metrics");
    // m.def("get_metric_names", &meme::get_metrics_names, "Get names for all mesh metrics");
    // m.def("get_metrics_per_tri", &meme::get_metrics_per_tri, "Get mesh metrics per triangle");
    // m.def(
    //     "get_metric_names_per_tri",
    //     &meme::get_metrics_names_per_tri,
    //     "Get names for all per-triangle metrics");
    // m.def(
    //     "get_relative_edge_lenghts",
    //     &meme::get_relative_edge_lengths,
    //     "Get edge lengths realtive to bbox diagonal");
    m.def(
        "wmtk",
        [](const py::dict& obj) {
            // py::scoped_ostream_redirect ostream(
            //     std::cout,
            //     py::module_::import("sys").attr("stdout"));
            // py::scoped_estream_redirect estream(
            //     std::cerr,
            //     py::module_::import("sys").attr("stderr"));
            try {
                wmtk_wrapper(obj);
            } catch (const std::exception& e) {
                std::cout << "Error: " << e.what() << std::endl;
            }
        },
        "Wildmeshing-Toolkit application");
}