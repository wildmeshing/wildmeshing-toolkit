#include <Eigen/Core>
template <typename Scalar>
inline bool is_close(
    const Scalar a,
    const Scalar b,
    const Scalar rtol = Eigen::NumTraits<Scalar>::dummy_precision(),
    const Scalar atol = Eigen::NumTraits<Scalar>::epsilon())
{
    return std::abs(a - b) <= (atol + rtol * std::abs(b));
}