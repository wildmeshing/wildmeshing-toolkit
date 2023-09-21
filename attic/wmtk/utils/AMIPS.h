#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>

namespace wmtk {
double AMIPS_energy(const std::array<double, 12>& T);
void AMIPS_jacobian(const std::array<double, 12>& T, Eigen::Vector3d& result_0);
void AMIPS_hessian(const std::array<double, 12>& T, Eigen::Matrix3d& result_0);
template <typename rational, typename dtype>
double AMIPS_energy_rational_p3(const std::array<dtype, 12>& T){
     std::array<rational, 12> r_T;
    for (int j = 0; j < 12; j++) r_T[j] = T[j];
    const rational twothird = rational(2) / rational(3);
    auto tmp = ((-r_T[1 + 2] + r_T[1 + 5]) * r_T[1 + 1] + r_T[1 + 2] * r_T[1 + 7] +
                (r_T[1 + -1] - r_T[1 + 5]) * r_T[1 + 4] - r_T[1 + -1] * r_T[1 + 7]) *
                   r_T[1 + 9] +
               ((r_T[1 + 2] - r_T[1 + 5]) * r_T[1 + 0] - r_T[1 + 2] * r_T[1 + 6] +
                (-r_T[1 + -1] + r_T[1 + 5]) * r_T[1 + 3] + r_T[1 + -1] * r_T[1 + 6]) *
                   r_T[1 + 10] +
               (-r_T[1 + 2] * r_T[1 + 7] + (-r_T[1 + 8] + r_T[1 + 5]) * r_T[1 + 4] +
                r_T[1 + 8] * r_T[1 + 7]) *
                   r_T[1 + 0] +
               (r_T[1 + 2] * r_T[1 + 6] + (r_T[1 + 8] - r_T[1 + 5]) * r_T[1 + 3] -
                r_T[1 + 8] * r_T[1 + 6]) *
                   r_T[1 + 1] +
               (r_T[1 + 3] * r_T[1 + 7] - r_T[1 + 4] * r_T[1 + 6]) * (r_T[1 + -1] - r_T[1 + 8]);
    if (tmp == 0) return std::numeric_limits<double>::infinity();

    auto res_r =
        rational(27) / 16 * pow(tmp, -2) *
        pow(r_T[1 + 9] * r_T[1 + 9] +
                (-twothird * r_T[1 + 0] - twothird * r_T[1 + 3] - twothird * r_T[1 + 6]) *
                    r_T[1 + 9] +
                r_T[1 + 10] * r_T[1 + 10] +
                (-twothird * r_T[1 + 1] - twothird * r_T[1 + 4] - twothird * r_T[1 + 7]) *
                    r_T[1 + 10] +
                r_T[1 + 0] * r_T[1 + 0] +
                (-twothird * r_T[1 + 3] - twothird * r_T[1 + 6]) * r_T[1 + 0] +
                r_T[1 + 1] * r_T[1 + 1] +
                (-twothird * r_T[1 + 4] - twothird * r_T[1 + 7]) * r_T[1 + 1] +
                r_T[1 + 2] * r_T[1 + 2] +
                (-twothird * r_T[1 + -1] - twothird * r_T[1 + 8] - twothird * r_T[1 + 5]) *
                    r_T[1 + 2] +
                r_T[1 + 3] * r_T[1 + 3] - twothird * r_T[1 + 3] * r_T[1 + 6] +
                r_T[1 + 4] * r_T[1 + 4] - twothird * r_T[1 + 4] * r_T[1 + 7] +
                r_T[1 + 5] * r_T[1 + 5] +
                (-twothird * r_T[1 + -1] - twothird * r_T[1 + 8]) * r_T[1 + 5] -
                twothird * r_T[1 + -1] * r_T[1 + 8] + r_T[1 + -1] * r_T[1 + -1] +
                r_T[1 + 8] * r_T[1 + 8] + r_T[1 + 6] * r_T[1 + 6] + r_T[1 + 7] * r_T[1 + 7],
            3);
    return res_r.to_double();
}
template <typename rational>
double AMIPS_energy_stable_p3(const std::array<double, 12>& T)
{
    auto res = AMIPS_energy(T);

    if (res < 1e8 && res > 2) {
        return std::pow(res, 3);
    } else {
        return AMIPS_energy_rational_p3<rational>(T);
    }
   
}
} // namespace wmtk
