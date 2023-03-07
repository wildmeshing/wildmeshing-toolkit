#pragma once
#include <wmtk/utils/Displacement.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <nlohmann/json.hpp>
#include <sec/envelope/SampleEnvelope.hpp>
namespace adaptive_tessellation {
enum class ENERGY_TYPE { AMIPS, SYMDI, EDGE_LENGTH, EDGE_QUADRATURE };
enum class EDGE_LEN_TYPE { LINEAR2D, LINEAR3D, N_IMPLICIT_POINTS, PT_PER_PIXEL, MIPMAP, ACCURACY };
struct Parameters
{
    using json = nlohmann::json;
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

public:
    json js_log;
    std::string m_output_folder = "./";
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

    // this need to be a function acting on the uv domain [0,1]
    std::function<Eigen::Vector3d(const double&, const double&)>
        m_adaptive_tessellation_displacement =
            [](const double& u, const double& v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 0.);
        return p;
    }; // used for heuristic split, collapse. Default to return (u,v,0)

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

    wmtk::MipMap m_mipmap;

    std::function<double(const std::size_t&, const std::size_t&)> m_get_length;

    double m_accuracy_threshold = 0.01;
    bool m_accuracy = 0;

    std::shared_ptr<wmtk::Displacement> m_displacement;
};
} // namespace adaptive_tessellation
