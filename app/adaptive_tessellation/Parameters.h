#pragma once
#include <wmtk/utils/Displacement.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <sec/envelope/SampleEnvelope.hpp>
using namespace wmtk;
namespace adaptive_tessellation {
enum class ENERGY_TYPE { AMIPS, SYMDI, EDGE_LENGTH, EDGE_QUADRATURE, AREA_QUADRATURE };
enum class EDGE_LEN_TYPE {
    LINEAR2D,
    LINEAR3D,
    N_IMPLICIT_POINTS,
    PT_PER_PIXEL,
    MIPMAP,
    ACCURACY,
    AREA_ACCURACY
};
struct Parameters
{
    using json = nlohmann::json;
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

public:
    json js_log;
    std::string m_output_folder = "./";
    std::shared_ptr<spdlog::logger> ATlogger =
        wmtk::make_json_file_logger("ATlogger", m_output_folder + "/runtime.log", true);
    // default envelop use_exact = true
    sample_envelope::SampleEnvelope m_envelope;
    bool m_has_envelope = false;
    double m_eps = 0.0; // envelope size default to 0.0
    bool m_bnd_freeze = false; // freeze boundary default to false

    double m_target_l = -1.; // targeted edge length
    double m_target_lr = 5e-2; // targeted relative edge lengths

    // Energy Assigned to undefined energy
    // TODO: why not the max double?
    const double MAX_ENERGY = std::numeric_limits<double>::infinity();
    double m_max_energy = -1;
    double m_stop_energy = 5;
    Eigen::Vector2d m_gradient = Eigen::Vector2d(0., 0.);
    std::unique_ptr<wmtk::Energy> m_energy;

    std::function<Eigen::RowVector2d(const Eigen::RowVector2d&)> m_get_closest_point =
        [](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        return p;
    }; // dafault is to return the current point. Legacy, was once used for boundary projection

    // takes 2 DScalar and returns z coordinates in DScalar
    // this is directly used by autodiff for taking Gradient and Hessian
    std::function<DScalar(const DScalar&, const DScalar&)> m_get_z;
    // takes 2 doubles and cast into DScalar. Then use the obtained z coordinate to consturct
    // directly a Vector3d
    // This is used to get a Vector3d for functions that requires 3d coordinates but no need for
    // taking gradients or hessian
    std::function<Eigen::Vector3d(const double&, const double&)> m_project_to_3d =
        [&](const double& u, const double& v) -> Eigen::Vector3d {
        DiffScalarBase::setVariableCount(2);
        auto z = this->m_get_z(DScalar(u), DScalar(v)).getValue();
        return Eigen::Vector3d(u, v, z);
    };

    wmtk::Boundary m_boundary; // stores boundary information
    bool m_boundary_parameter = true; // use boundary single variable parameterization

    WrappingMode m_wrapping_mode = WrappingMode::MIRROR_REPEAT;

    std::function<std::pair<int, int>(const double&, const double&)> m_image_get_coordinate;

    wmtk::Image m_image;
    std::array<std::filesystem::path, 2> m_position_normal_paths;
    wmtk::MipMap m_mipmap;

    std::function<double(const TriMesh::Tuple& edge_tuple)> m_get_length;

    double m_quality_threshold = 0.01;
    double m_accuracy_threshold = 0.001;
    double m_accuracy_safeguard_ratio = 1.1;

    EDGE_LEN_TYPE m_edge_length_type = EDGE_LEN_TYPE::ACCURACY;
    SAMPLING_MODE m_sampling_mode = SAMPLING_MODE::BICUBIC;
    DISPLACEMENT_MODE m_displacement_mode = DISPLACEMENT_MODE::PLANE;
    std::shared_ptr<wmtk::Displacement> m_displacement;
    double m_scale = 1.0;
    Eigen::Matrix<double, 1, 3> m_offset;

    bool m_swap_using_valence = 1;
    bool m_split_absolute_error_metric = 1;

    // early stopping after n operations. default to infinity
    int m_early_stopping_number = std::numeric_limits<size_t>::max();
    // only operate to modify topologies
    bool m_ignore_embedding = false;
    // used for scaling the height map
    double m_normalization_scale = 1.0;
    bool m_do_not_output = false;

public:
    void log(
        const nlohmann::json& js,
        bool flush = false) // flush should force file output immediately, but will be slow for
                            // per-operation things
    {
        std::cout << js.dump() << std::endl;
        ATlogger->error(js.dump());


        if (flush) {
            ATlogger->flush();
        }
    }

    // log that always writes to file immediately beause it's flushing
    void log_flush(const nlohmann::json& js) { log(js, true); }
};
} // namespace adaptive_tessellation
