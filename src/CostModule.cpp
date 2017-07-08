#include "CostModule.h"
#include <cppad/cppad.hpp>

using CppAD::AD;

AD<double> CostModule::GetCost(State& prev, State& next)
{
	return weight_ * CppAD::pow(CalculateCost(prev, next), 2);
}

AD<double> CTE_CostModule::CalculateCost(State& prev, State& next)
{
	return prev.cte_.val_;
}

AD<double> EPSI_CostModule::CalculateCost(State& prev, State& next)
{
	return prev.epsi_.val_;
}

AD<double> V_CostModule::CalculateCost(State& prev, State& next)
{
	return prev.v_.val_ - settings_.tar_v;
}

AD<double> Steering_CostModule::CalculateCost(State& prev, State& next)
{
	return prev.delta_.val_;
}

AD<double> SteeringDelta_CostModule::CalculateCost(State& prev, State& next)
{
	return next.delta_.val_ - prev.delta_.val_;
}
