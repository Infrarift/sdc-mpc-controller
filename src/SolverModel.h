#pragma once
#include "State.h"
#include "Eigen-3.3/Eigen/Core"
#include "MPC_Settings.h"

using CppAD::AD;

class SolverModel
{
public:
	int state_index_;
	MPC_Settings settings_;
	virtual AD<double> Solve(State& prev_state, State& next_state, Eigen::VectorXd& coeffs, double dt) = 0;
};

class X_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};

class Y_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};

class PSI_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};

class V_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};

class CTE_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};

class EPSI_Model : public SolverModel
{
public:
	AD<double> Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt) override;
};