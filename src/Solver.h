#pragma once
#include "Eigen-3.3/Eigen/Core"
#include "SolverModel.h"
#include <cppad/cppad.hpp>

using CppAD::AD;

class Solver
{
	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

	MPC_Settings settings_;
	std::vector<SolverModel*> _solverModels;
	X_Model x_model_;
	Y_Model y_model_;
	PSI_Model psi_model_;
	V_Model v_model_;
	CTE_Model cte_model_;
	EPSI_Model epsi_model_;

public:

	explicit Solver(MPC_Settings settings);
	void CreateAndAddNewState(SolverModel& model, int N);
	void Solve(ADvector& fg, State& prev_state, State& next_state, Eigen::VectorXd coeffs, int index_offset, double dt);

};
