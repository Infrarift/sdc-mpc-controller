#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 8;
double dt = 0.15;

const double Lf = 2.67;

// Target Values for the Errors
double tar_cte = 0;
double tar_epsi = 0;
double tar_v = 50;

// Indexes on the 1D vector (for readability)
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval
{
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR (AD<double>) ADvector;

	CppAD::AD<double> CalculateCost(const double cte_weight, CppAD::AD<double> error)
	{
		return cte_weight * CppAD::pow(error, 2);
	}

	void operator()(ADvector& fg, const ADvector& vars)
	{
		// TODO: implement MPC
		// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
		// NOTE: You'll probably go back and forth between this function and
		// the Solver function below.

		fg[0] = 0;

		// Define weights for different terms of objective
		const double cte_weight = 200;
		const double epsi_weight = 15000;
		const double v_weight = 1;
		const double actuator_cost_weight = 100000;
		const double actuator_cost_weight_a = 0;
		const double change_steer_cost_weight = 100000;
		const double change_accel_cost_weight = 0;

		// Objective term 1: Keep close to reference values
		for (size_t t = 0; t < N; ++t)
		{
			fg[0] += CalculateCost(cte_weight, vars[cte_start + t] - tar_cte);
			fg[0] += CalculateCost(epsi_weight, vars[epsi_start + t] - tar_epsi);
			fg[0] += CalculateCost(v_weight, vars[v_start + t] - tar_v);
		}

		// Objective term 2:  Avoid to actuate, as much as possible
		for (size_t t = 0; t < N - 1; ++t)
		{
			fg[0] += actuator_cost_weight * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += actuator_cost_weight_a * CppAD::pow(vars[a_start + t], 2);
		}

		// Objective term 3:  Enforce actuators smoothness in change
		for (size_t t = 0; t < N - 2; ++t)
		{
			fg[0] += change_steer_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += change_accel_cost_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Initial constraints
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		for (size_t t = 1; t < N; ++t)
		{

			// Previous State
			auto p_t = t - 1;
			AD <double > x_0 = vars[x_start + p_t];
			AD <double > y_0 = vars[y_start + p_t];
			AD <double > psi_0 = vars[psi_start + p_t];
			AD <double > v_0 = vars[v_start + p_t];
			AD <double > cte_0 = vars[cte_start + p_t];
			AD <double > epsi_0 = vars[epsi_start + p_t];

			// Current State
			AD <double > x_1 = vars[x_start + t];
			AD <double > y_1 = vars[y_start + t];
			AD <double > psi_1 = vars[psi_start + t];
			AD <double > v_1 = vars[v_start + t];
			AD <double > cte_1 = vars[cte_start + t];
			AD <double > epsi_1 = vars[epsi_start + t];

			AD <double > delta_0 = vars[delta_start + p_t];
			AD <double > a_0 = vars[a_start + p_t];

			AD <double > f_0 = coeffs[0] +
				coeffs[1] * x_0 +
				coeffs[2] * x_0 * x_0 +
				coeffs[3] * x_0 * x_0 * x_0;

			AD <double > psides_0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x_0 + 3 * coeffs[3] * x_0 * x_0);

			// Setup other model constraints
			fg[1 + x_start + t] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
			fg[1 + y_start + t] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
			fg[1 + psi_start + t] = psi_1 - (psi_0 - v_0 * delta_0 / Lf * dt);
			fg[1 + v_start + t] = v_1 - (v_0 + a_0 * dt);
			fg[1 + cte_start + t] = cte_1 - (f_0 - y_0 + (v_0 * CppAD::sin(epsi_0) * dt));
			fg[1 + epsi_start + t] = epsi_1 - (psi_0 - psides_0 - v_0 * delta_0 / Lf * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC()
{
}

MPC::~MPC()
{
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR (
double
	)
	Dvector;

	// TODO: Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	//
	// 4 * 10 + 2 * 9

	int n_state = 6;
	int n_actuators = 2;

	size_t n_vars = n_state * N + n_actuators * (N - 1);
	size_t n_constraints = n_state * N;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);

	for (int i = 0; i < n_vars; i++)
	{
		vars[i] = 0;
	}

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	for (size_t i = 0; i < delta_start; ++i)
	{
		vars_lowerbound[i] = -numeric_limits<float>::max();
		vars_upperbound[i] = +numeric_limits<float>::max();
	}

	// Steering
	double max_degree = 25;
	double max_radians = max_degree * M_PI / 180;
	for (size_t i = delta_start; i < a_start; ++i)
	{
		vars_lowerbound[i] = -max_radians;
		vars_upperbound[i] = +max_radians;
	}

	// Throttle
	double max_acceleration_value = 1.0;
	for (size_t i = a_start; i < n_vars; ++i)
	{
		vars_lowerbound[i] = -max_acceleration_value;
		vars_upperbound[i] = +max_acceleration_value;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i = 0; i < n_constraints; i++)
	{
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	for (int i = 0, m = state.size(); i < m; i++)
	{
		auto j = i * N;
		constraints_lowerbound[j] = state[i];
		constraints_upperbound[j] = state[i];
		vars[j] = state[i];
	}

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.

	// Return the first actuator values. The variables can be accessed with `solution.x[i]`.
	vector<double> result;
	result.push_back(solution.x[delta_start]);
	result.push_back(solution.x[a_start]);

	// Add "future" solutions (where MPC is going)
	for (size_t i = 0; i < N; ++i) {
		result.push_back(solution.x[x_start + i]);
		result.push_back(solution.x[y_start + i]);
	}

	return result;
}
