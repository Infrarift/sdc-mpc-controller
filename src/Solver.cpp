#include "Solver.h"

Solver::Solver(MPC_Settings settings)
{
	settings_ = settings;
	auto N = settings_.N;

	CreateAndAddNewState(x_model_, 0);
	CreateAndAddNewState(y_model_, N);
	CreateAndAddNewState(psi_model_, N);
	CreateAndAddNewState(v_model_, N);
	CreateAndAddNewState(cte_model_, N);
	CreateAndAddNewState(epsi_model_, N);
}

void Solver::CreateAndAddNewState(SolverModel& model, int N)
{
	auto newIndex = N == 0 ? 0 : (*solverModels_.back()).state_index_ + N;
	model.settings_ = settings_;
	model.state_index_ = newIndex;
	solverModels_.push_back(&model);
}

void Solver::Solve(ADvector& fg, State& prev_state, State& next_state, Eigen::VectorXd coeffs, int index_offset, double dt)
{
	for (auto i = 0; i < solverModels_.size(); i++)
		fg[solverModels_[i]->state_index_ + index_offset] = solverModels_[i]->Solve(prev_state, next_state, coeffs, dt);
}