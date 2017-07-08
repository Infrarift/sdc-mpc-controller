#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "CostModule.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "MPC_Settings.h"

using namespace std;

class MPC {

	typedef CPPAD_TESTVECTOR(double) Dvector;

	CTE_CostModule cte_cost;
	V_CostModule v_cost;
	EPSI_CostModule epsi_cost;
	Steering_CostModule steer_cost;
	SteeringDelta_CostModule steer_delta_cost;
	State state_;

	void AddCost(CostModule* cost_module, double weight, int max_index);
	Dvector CreateVector(int dimensions, double default_value);
	void FillVector(Dvector& vector, int start_index, int end_index, double value);
	void ExtractWaypoints(CppAD::ipopt::solve_result<Dvector> solution);

public:
	MPC();
	virtual ~MPC();
	MPC_Settings settings_;
	vector<CostModule*> cost_modules_;
	vector<double> ai_waypoints_x_;
	vector<double> ai_waypoints_y_;
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
#endif /* MPC_H */
