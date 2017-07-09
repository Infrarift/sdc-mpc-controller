#pragma once
#include "State.h"
#include <cppad/cppad.hpp>
#include "MPC_Settings.h"

using CppAD::AD;

class CostModule
{
public:
	double weight_;
	size_t max_index_;
	MPC_Settings settings_;
	virtual AD<double> CalculateCost(State& prev, State& next) = 0;
	AD<double> GetCost(State& prev, State& next);
};

class CTE_CostModule : public CostModule
{
public:
	AD<double> CalculateCost(State& prev, State& next) override;
};

class EPSI_CostModule : public CostModule
{
public:
	AD<double> CalculateCost(State& prev, State& next) override;
};

class V_CostModule : public CostModule
{
public:
	AD<double> CalculateCost(State& prev, State& next) override;
};

class Steering_CostModule : public CostModule
{
public:
	AD<double> CalculateCost(State& prev, State& next) override;
};

class SteeringDelta_CostModule : public CostModule
{
public:
	AD<double> CalculateCost(State& prev, State& next) override;
};

