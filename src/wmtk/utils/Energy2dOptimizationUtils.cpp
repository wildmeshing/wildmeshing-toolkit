#include "Energy2dOptimizationUtils.h"
using namespace wmtk;
// check every triangle in the assembles with the dofx whether any trinagle is flipped
// if it is boundary vertex, dofx is t but needs to be converted to uv
// since it's just inversion check, no need for autodiff notation
// if failed inversion check, return false
// if passed inversion check, return true
bool wmtk::inversion_check_with_dofx(
    const wmtk::Boundary& boundary_mapping,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    const wmtk::DofVector& dofx)
{
    for (auto& nminfo : nminfos) {
        for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
            Eigen::Vector2d A, B, C;
            if (dofx.size() == 1) {
                A = boundary_mapping.t_to_uv(nminfo.curve_id, dofx(0));
            } else
                A = dofx;
            B << nminfo.neighbors(i, 0), nminfo.neighbors(i, 1);
            C << nminfo.neighbors(i, 2), nminfo.neighbors(i, 3);
            auto res = igl::predicates::orient2d(A, B, C);
            if (res != igl::predicates::Orientation::POSITIVE) return false;
        }
    }
    return true;
}

void wmtk::optimization_state_update(
    const wmtk::Energy& energy_def,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    const wmtk::Boundary& boundary_mapping,
    wmtk::State& state)
{
    // assume no inversion
    assert(inversion_check_with_dofx(boundary_mapping, nminfos, state.dofx));
    auto total_energy = 0.;
    Eigen::Vector2d total_jac = Eigen::Vector2d::Zero();
    Eigen::Matrix2d total_hess = Eigen::Matrix2d::Zero();
    for (auto& nminfo : nminfos) {
        for (auto i = 0; i < nminfo.neighbors.rows(); i++) {
            // set State
            // pass the state energy
            state.two_opposite_vertices = nminfo.neighbors.row(i);
            state.scaling = nminfo.target_length;
            state.idx = nminfo.facet_ids(i);
            assert(boundary_mapping.num_curves() > 0);
            DofsToPositions dofs_to_pos(boundary_mapping, nminfo.curve_id);
            energy_def.eval(state, dofs_to_pos);
            total_energy += state.value;
            assert(!std::isnan(total_energy));
            total_jac += state.gradient;
            total_hess += state.hessian;
        }
    }
    state.value = total_energy;
    state.gradient = total_jac;
    state.hessian = total_hess;
}

// keep the current dofx in state
void wmtk::linesearch_update_dofx(
    wmtk::DofVector& dir,
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    wmtk::State& state,
    int max_iter)
{
    auto lr = 0.5;

    DofVector fallback_dofx = state.dofx;
    DofVector old_dofx;
    double old_energy = state.value;
    while (lr > std::numeric_limits<double>::denorm_min() && state.value >= old_energy) {
        old_dofx = state.dofx;
        // update dofx in state
        state.dofx = old_dofx + lr * dir;
        if (inversion_check_with_dofx(boundary_mapping, nminfos, state.dofx)) {
            // can only compute energy when the triangle isn't flipped
            optimization_state_update(energy_def, nminfos, boundary_mapping, state);
            if (state.value < old_energy) return;
        }
        lr /= 2.;
    }
    if (state.value >= old_energy) state.dofx = fallback_dofx;
    return;
}

// assume the state is updated
wmtk::DofVector wmtk::newton_direction_2d_with_state(wmtk::State& state)
{
    Eigen::Vector2d dir = state.hessian.ldlt().solve(state.gradient);
    if (!(state.gradient)
             .isApprox(
                 state.hessian * dir,
                 1e-6)) // a hacky PSD trick. TODO: change this.
    {
        logger().info("gradient descent instead.");
        dir = state.gradient;
    }
    return -dir;
}

// assume the state is updated
wmtk::DofVector wmtk::gradient_descent_direction_2d_with_state(State& state)
{
    logger().info("######### using gradient descent #########");
    return -state.gradient;
}

void wmtk::optimization_dofx_update(
    const wmtk::Energy& energy_def,
    const wmtk::Boundary& boundary_mapping,
    const std::vector<wmtk::NewtonMethodInfo>& nminfos,
    wmtk::State& state)
{
    wmtk::State old_state = state;
    auto line_search_iters = 12;
    optimization_state_update(energy_def, nminfos, boundary_mapping, state);
    if (state.value == std::numeric_limits<double>::infinity()) {
        logger().info("////////// energy is infinity. skip.");
        return;
    }
    // get line search direction
    DofVector dir(2), search_dir(2); // size() == 1 if it's boundary, ow, size() == 2
    // first try newton's method
    dir = wmtk::newton_direction_2d_with_state(state);
    if (state.dofx.size() == 1) {
        search_dir.resize(1);
        search_dir(0) = dir(0);
    } else
        search_dir = dir;
    wmtk::linesearch_update_dofx(
        search_dir,
        energy_def,
        boundary_mapping,
        nminfos,
        state,
        line_search_iters);

    // if newton's method do not update the dofx, then try gradien descent
    if ((state.dofx - old_state.dofx).squaredNorm() < std::numeric_limits<double>::denorm_min()) {
        state = old_state;
        dir = wmtk::gradient_descent_direction_2d_with_state(state);
        if (state.dofx.size() == 1) {
            search_dir.resize(1);
            search_dir(0) = dir(0);
        } else
            search_dir = dir;
        wmtk::linesearch_update_dofx(
            search_dir,
            energy_def,
            boundary_mapping,
            nminfos,
            state,
            line_search_iters);
    }
}
