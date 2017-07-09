#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "CostModule.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "MPC_Settings.h"

using namespace std;

class MPC
{
	typedef CPPAD_TESTVECTOR(double) Dvector;

	CTE_CostModule cte_cost_;
	V_CostModule v_cost_;
	EPSI_CostModule epsi_cost_;
	Steering_CostModule steer_cost_;
	SteeringDelta_CostModule steer_delta_cost_;
	State state_;

	size_t n_states_;
	size_t n_actuators_;
	size_t n_vars_;
	size_t n_constraints_;

	void AddCost(CostModule* cost_module, double weight, int max_index);
	Dvector CreateVector(int dimensions, double default_value);
	void FillVector(Dvector& vector, int start_index, int end_index, double value);
	void ExtractWaypoints(CppAD::ipopt::solve_result<Dvector> solution, Eigen::VectorXd poly);
	void AddInitialStateConstraints(BasicState& state, Eigen::VectorXd coeffs, Dvector& vars1, Dvector& vars2, Dvector& vars3);


	double PolyEval(Eigen::VectorXd coeffs, double x) const;
	static Eigen::VectorXd PolyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
	void CreateWayPoints(Eigen::VectorXd& x_wp, Eigen::VectorXd& y_wp, const vector<double>& x_mp, const vector<double>& y_mp, double x, double y, double psi) const;

public:
	explicit MPC(MPC_Settings settings);
	virtual ~MPC();
	MPC_Settings settings_;
	vector<CostModule*> cost_modules_;
	vector<double> ai_waypoints_x_;
	vector<double> ai_waypoints_y_;
	vector<double> map_waypoints_x_;
	vector<double> map_waypoints_y_;
	vector<double> Solve(BasicState state, vector<double> pts_x, vector<double> pts_y);
};
#endif /* MPC_H */
