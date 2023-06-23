#pragma once

#include <Eigen/Dense>

#include <cassert>
#include <vector>

namespace wmtk {

class Quadrature
{
public:
    using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
    size_t size() const { return m_weights.size(); }

    size_t dimension() const { return m_dimension; }

    void set_dimension(size_t new_dim)
    {
        m_dimension = new_dim;
        m_points.clear();
        m_weights.clear();
    }

    void resize(size_t new_size)
    {
        assert(dimension() != 0);
        m_points.resize(dimension() * new_size);
        m_weights.resize(new_size);
    }

    void clear() {
        m_points.clear();
        m_weights.clear();
    }

    const Eigen::Map<const RowMatrixXd> points() const
    {
        return {
            m_points.data(),
            static_cast<Eigen::Index>(size()),
            static_cast<Eigen::Index>(dimension())};
    };

    Eigen::Map<RowMatrixXd> points()
    {
        return {
            m_points.data(),
            static_cast<Eigen::Index>(size()),
            static_cast<Eigen::Index>(dimension())};
    }

    const Eigen::Map<const Eigen::VectorXd> weights() const
    {
        return {m_weights.data(), static_cast<Eigen::Index>(size())};
    };

    Eigen::Map<Eigen::VectorXd> weights()
    {
        return {m_weights.data(), static_cast<Eigen::Index>(size())};
    }

private:
    size_t m_dimension = 0;
    std::vector<double> m_points;
    std::vector<double> m_weights;
};

} // namespace wmtk
