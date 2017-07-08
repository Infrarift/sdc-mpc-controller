#include "SolverModel.h"

using CppAD::AD;
using CppAD::cos;
using CppAD::sin;
using CppAD::pow;
using CppAD::atan;

AD<double> X_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	return next.x_.val_ - (prev.x_.val_ + prev.v_.val_ * cos(prev.psi_.val_) * dt);
}

AD<double> Y_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	return next.y_.val_ - (prev.y_.val_ + prev.v_.val_ * sin(prev.psi_.val_) * dt);
}

AD<double> PSI_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	return next.psi_.val_ - (prev.psi_.val_ - prev.v_.val_ * prev.delta_.val_ / settings_.Lf * dt);
}

AD<double> V_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	return next.v_.val_ - (prev.v_.val_ + prev.a_.val_ * dt);
}

AD<double> CTE_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	AD <double> f = coeffs[0] +
		coeffs[1] * prev.x_.val_ +
		coeffs[2] * pow(prev.x_.val_, 2) +
		coeffs[3] * pow(prev.x_.val_, 3);

	return next.cte_.val_ - (f - prev.y_.val_ + (prev.v_.val_ * sin(prev.epsi_.val_) * dt));
}

AD<double> EPSI_Model::Solve(State& prev, State& next, Eigen::VectorXd& coeffs, double dt)
{
	AD <double> psides = atan(coeffs[1] +
		2 * coeffs[2] * prev.x_.val_ +
		3 * coeffs[3] * prev.x_.val_ * prev.x_.val_);

	return next.epsi_.val_ - (prev.psi_.val_ - psides - prev.v_.val_ * prev.delta_.val_ / settings_.Lf * dt);
}