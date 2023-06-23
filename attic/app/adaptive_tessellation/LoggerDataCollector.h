#pragma once

// clang-format off
#include <chrono>
#include <spdlog/stopwatch.h>
// clang-format on
#include "AdaptiveTessellation.h"

namespace adaptive_tessellation {

class LoggerDataCollector
{
    size_t peak_memory_ = -1; // peak memory in bytes
    size_t num_faces_ = -1;
    size_t num_vertices_ = -1;
    std::vector<float> triangle_energies_;
    std::vector<double> triangle_areas_;
    // std::vector<double> triangle_min_angles_;
    std::vector<double> edge_lengths_;

    // mutable igl::Timer timer_;
    std::unique_ptr<spdlog::stopwatch> timer_;
    double runtime_ = -1;

public:
    /**
     * @brief Collect logging data from the mesh.
     *
     * Computes the following things: peak memory, number of faces, number of vertices, edge length
     * statistics, energy statistic, area statistic. All statistics contain min, max, mean, median,
     * and standard deviation.
     */
    void evaluate_mesh(const AdaptiveTessellation& mesh);

    void start_timer() { timer_ = std::make_unique<spdlog::stopwatch>(); }
    void stop_timer()
    {
        if (timer_ != nullptr) {
            runtime_ = timer_->elapsed().count();
        } else {
            wmtk::logger().warn("stop_timer() was called but timer was never started");
        }
    }

    double time_in_seconds() const { return runtime_; }

    /**
     * @brief Log collected data using the mesh logger.
     *
     * If timer was not started or stopped, -1 will be printed for the runtime.
     *
     * @param mesh
     * @param log_name name of the log message
     */
    void log_json(
        const AdaptiveTessellation& mesh,
        const std::string& log_name,
        const bool with_vectors = false) const;

    void log_json_verbose(const AdaptiveTessellation& mesh, const std::string& log_name);

private:
    nlohmann::json general_info_to_json() const;

    nlohmann::json vectors_to_json() const;
};
} // namespace adaptive_tessellation