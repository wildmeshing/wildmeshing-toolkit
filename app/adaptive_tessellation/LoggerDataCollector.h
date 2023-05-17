#pragma once

#include <igl/Timer.h>
#include "AdaptiveTessellation.h"
#include "wmtk/utils/getRSS.h"

namespace adaptive_tessellation {

class LoggerDataCollector
{
    class StatisticsObj
    {
        double min_ = std::numeric_limits<double>::max();
        double max_ = -std::numeric_limits<double>::max();
        double mean_ = -std::numeric_limits<double>::max();
        double median_ = -std::numeric_limits<double>::max();
        double std_dev_ = -std::numeric_limits<double>::max();

    public:
        void compute_statistics(std::vector<double>& data)
        {
            std::sort(data.begin(), data.end());

            min_ = data[0];
            max_ = data[data.size() - 1];
            median_ = data[(data.size() - 1) / 2];

            double sum = std::accumulate(data.begin(), data.end(), 0.0);
            mean_ = sum / data.size();

            std::vector<double> diff(data.size());
            std::transform(data.begin(), data.end(), diff.begin(), [this](double x) {
                return x - mean_;
            });
            const double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
            std_dev_ = std::sqrt(sq_sum / data.size());
        }

        double min() const { return min_; }
        double max() const { return max_; }
        double mean() const { return mean_; }
        double median() const { return median_; }
        double std_dev() const { return std_dev_; }
    };

    size_t peak_memory_ = -1; // peak memory in bytes
    size_t num_faces_ = -1;
    size_t num_vertices_ = -1;
    StatisticsObj edge_length_;
    StatisticsObj energy_;
    StatisticsObj area_;
    // StatisticsObj min_angle_;

    mutable igl::Timer timer_;
    bool timer_started_ = false;
    bool timer_stopped_ = false;

public:
    void evaluate_mesh(const AdaptiveTessellation& mesh)
    {
        using Tuple = wmtk::TriMesh::Tuple;

        peak_memory_ = wmtk::getPeakRSS();
        // face stuff
        {
            const auto faces = mesh.get_faces();
            num_faces_ = faces.size();

            std::vector<double> energy_vec;
            std::vector<double> area_vec;
            // std::vector<double> min_angle_vec;
            energy_vec.reserve(num_faces_);
            area_vec.reserve(num_faces_);
            // min_angle_vec.reserve(num_faces_);

            for (const Tuple& f : faces) {
                const auto vids = mesh.oriented_tri_vertices(f);
                Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
                for (size_t i = 0; i < 3; ++i) {
                    triangle.row(i) = mesh.vertex_attrs[vids[i].vid(mesh)].pos;
                }
                const double triangle_area = wmtk::polygon_signed_area(triangle);
                const double triangle_energy =
                    mesh.mesh_parameters.m_displacement->get_error_per_triangle(triangle);

                area_vec.emplace_back(triangle_area);
                energy_vec.emplace_back(triangle_energy);
            }

            area_.compute_statistics(area_vec);
            energy_.compute_statistics(energy_vec);
        }
        // edge stuff
        {
            const auto edges = mesh.get_edges();

            std::vector<double> edge_length_vec;
            edge_length_vec.reserve(edges.size());

            for (const Tuple e : edges) {
                const double l = mesh.get_length3d(e);
                edge_length_vec.emplace_back(l);
            }
            edge_length_.compute_statistics(edge_length_vec);
        }
        // vertex stuff
        {
            const auto vertices = mesh.get_vertices();
            num_vertices_ = vertices.size();
        }
    }

    void start_timer()
    {
        timer_.start();
        timer_started_ = true;
    }
    void stop_timer()
    {
        timer_.stop();
        timer_stopped_ = true;
    }

    double time_in_seconds() const { return timer_.getElapsedTimeInSec(); }

    void log_json(const AdaptiveTessellation& mesh, const std::string& log_name) const
    {
        const double runtime = (timer_started_ && timer_stopped_) ? time_in_seconds() : -1;

        mesh.mesh_parameters.log(
            {{log_name,
              {{"peak_memory", peak_memory_},
               {"num_faces", num_faces_},
               {"num_vertices", num_vertices_},
               {"runtime", runtime},
               {"edge_length",
                {{"min", edge_length_.min()},
                 {"max", edge_length_.max()},
                 {"mean", edge_length_.mean()},
                 {"median", edge_length_.median()},
                 {"std_dev", edge_length_.std_dev()}}},
               {"energy",
                {{"min", energy_.min()},
                 {"max", energy_.max()},
                 {"mean", energy_.mean()},
                 {"median", energy_.median()},
                 {"std_dev", energy_.std_dev()}}},
               {"area",
                {{"min", area_.min()},
                 {"max", area_.max()},
                 {"mean", area_.mean()},
                 {"median", area_.median()},
                 {"std_dev", area_.std_dev()}}}}}});
    }
};
} // namespace adaptive_tessellation