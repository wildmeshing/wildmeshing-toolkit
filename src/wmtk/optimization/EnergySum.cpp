#include "EnergySum.hpp"

#include <wmtk/utils/orient.hpp>

namespace wmtk::optimization {

void EnergySum::add_energy(const std::shared_ptr<Problem>& energy, const double weight)
{
    m_energies.push_back(energy);
    m_weights.push_back(weight);
}

double EnergySum::value(const TVector& x)
{
    double res = 0;
    for (size_t i = 0; i < m_energies.size(); ++i) {
        res += m_weights[i] * m_energies[i]->value(x);
    }
    return res;
}

void EnergySum::gradient(const TVector& x, TVector& gradv)
{
    m_energies[0]->gradient(x, gradv);
    gradv *= m_weights[0];

    for (size_t i = 1; i < m_energies.size(); ++i) {
        TVector g;
        m_energies[i]->gradient(x, g);
        gradv += m_weights[i] * g;
    }
}

void EnergySum::hessian(const TVector& x, MatrixXd& hessian)
{
    m_energies[0]->hessian(x, hessian);
    hessian *= m_weights[0];

    for (size_t i = 1; i < m_energies.size(); ++i) {
        MatrixXd h;
        m_energies[i]->hessian(x, h);
        hessian += m_weights[i] * h;
    }
}

void EnergySum::solution_changed(const TVector& new_x) {}

bool EnergySum::is_step_valid(const TVector& x0, const TVector& x1)
{
    for (const auto& e : m_energies) {
        if (!e->is_step_valid(x0, x1)) {
            return false;
        }
    }

    return true;
}

} // namespace wmtk::optimization