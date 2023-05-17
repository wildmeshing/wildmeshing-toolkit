#pragma once
#include <igl/predicates/predicates.h>
#include <lagrange/utils/timing.h>
#include <Eigen/Core>
#include <optional>
#include "Energy2d.h"
#include "TriQualityUtils.hpp"
namespace wmtk {
bool inversion_check_with_dofx(
    const wmtk::Boundary& boundary_mapping,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    const DofVector& dofx);
void optimization_state_update(
    const wmtk::Energy& energy_def,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    const wmtk::Boundary& boundary_mapping,
    State& state);

// keep the current dofx in state
void linesearch_update_dofx(
    wmtk::DofVector& dir,
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const std::vector<NewtonMethodInfo>& nminfos,
    State& state,
    int max_iter);

// assume the state is updated
wmtk::DofVector newton_direction_2d_with_state(State& state);

// assume the state is updated
wmtk::DofVector gradient_descent_direction_2d_with_state(State& state);

void optimization_dofx_update(
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const std::vector<NewtonMethodInfo>& nminfos,
    State& state);
} // namespace wmtk