#include <iostream>
#include "MPC.h"
#include "State.h"
#include "Solver.h"
#include "CostModule.h"

using CppAD::AD;

class FG_eval
{
public:

	typedef CPPAD_TESTVECTOR (AD<double>) ADvector;

	Eigen::VectorXd coeffs;
	vector<CostModule*>* cost_modules_;
	MPC_Settings settings_;

	FG_eval(Eigen::VectorXd coeffs, vector<CostModule*>* cost_modules, MPC_Settings settings)
	{
		this->coeffs = coeffs;
		this->cost_modules_ = cost_modules;
		this->settings_ = settings;
	}

	void operator()(ADvector& fg, const ADvector& vars)
	{
		fg[0] = 0;

		auto previous_state = State(settings_.N);
		auto next_state = State(settings_.N);
		auto solver = Solver(settings_);

		// Set the initial state.
		previous_state.SetValues(vars, 0);
		for (auto i = 0, m = 5; i < m; i++)
		{
			auto state = *previous_state.states_[i];
			fg[state.start_index_ + 1] = state.val_;
		}

		for (auto t = 0; t < settings_.N; ++t)
		{
			previous_state.SetValues(vars, t);
			next_state.SetValues(vars, t + 1);

			// Solve state for each step.
			if (t < settings_.N - 1)
				solver.Solve(fg, previous_state, next_state, coeffs, t + 2, settings_.dt);

			// Solve cost for each step.
			for (int i = 0, m = cost_modules_->size(); i < m; i++)
			{
				if ((*cost_modules_)[i]->max_index_ > t)
					fg[0] += (*cost_modules_)[i]->GetCost(previous_state, next_state);
			}
		}
	}
};

MPC::MPC()
{
	settings_.N = 8;
	settings_.dt = 0.15;
	settings_.Lf = 2.67;
	settings_.tar_v = 50;

	settings_.cte_cost_w = 200;
	settings_.v_cost_w = 1;
	settings_.epsi_cost_w = 15000;
	settings_.steer_cost_w = 100000;
	settings_.steer_delta_cost_w = 100000;

	AddCost(&cte_cost, settings_.cte_cost_w, settings_.N);
	AddCost(&v_cost, settings_.v_cost_w, settings_.N);
	AddCost(&epsi_cost, settings_.epsi_cost_w, settings_.N);
	AddCost(&steer_cost, settings_.steer_cost_w, settings_.N - 1);
	AddCost(&steer_delta_cost, settings_.steer_delta_cost_w, settings_.N - 2);

	state_.Initialize(settings_.N);
}

MPC::~MPC()
{
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
	bool ok = true;
	int n_state = 6;
	int n_actuators = 2;

	size_t n_vars = n_state * settings_.N + n_actuators * (settings_.N - 1);
	size_t n_constraints = n_state * settings_.N;

	// Create variable and constraint vectors.
	auto vars = CreateVector(n_vars, 0);
	auto vars_lowerbound = CreateVector(n_vars, -numeric_limits<float>::max());
	auto vars_upperbound = CreateVector(n_vars, numeric_limits<float>::max());
	auto constraints_lowerbound = CreateVector(n_constraints, 0);
	auto constraints_upperbound = CreateVector(n_constraints, 0);

	// Set variable limits.
	FillVector(vars_lowerbound, state_.delta_.start_index_, state_.a_.start_index_, -0.436);
	FillVector(vars_upperbound, state_.delta_.start_index_, state_.a_.start_index_, 0.436);
	FillVector(vars_lowerbound, state_.a_.start_index_, n_vars, -1.0);
	FillVector(vars_upperbound, state_.a_.start_index_, n_vars, 1.0);

	// TODO: Extract this to initialize the first contraints better.
	for (size_t i = 0, m = state.size(); i < m; i++)
	{
		auto j = i * settings_.N;
		constraints_lowerbound[j] = state[i];
		constraints_upperbound[j] = state[i];
		vars[j] = state[i];
	}

	// Create solver object.
	FG_eval fg_eval(coeffs, &cost_modules_, settings_);

	std::string options;
	options += "Integer print_level  0\n";
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	options += "Numeric max_cpu_time          0.5\n";

	// Place to store the solution.
	CppAD::ipopt::solve_result<Dvector> solution;

	// Solve the problem.
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check Solution Values.
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Total Cost.
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// Return MPC Waypoints.
	ExtractWaypoints(solution);

	// Create a vector to return the actuator values.
	vector<double> result;
	result.push_back(solution.x[state_.delta_.start_index_]);
	result.push_back(solution.x[state_.a_.start_index_]);

	return result;
}

void MPC::AddCost(CostModule* cost_module, double weight, int max_index)
{
	cost_module->weight_ = weight;
	cost_module->max_index_ = max_index;
	cost_module->settings_ = settings_;
	cost_modules_.push_back(cost_module);
}

MPC::Dvector MPC::CreateVector(int dimensions, double default_value)
{
	Dvector vec(dimensions);
	FillVector(vec, 0, dimensions, default_value);
	return vec;
}

void MPC::FillVector(Dvector& vector, int start_index, int end_index, double value)
{
	for (size_t i = start_index; i < end_index; ++i)
		vector[i] = value;
}

void MPC::ExtractWaypoints(CppAD::ipopt::solve_result<Dvector> solution)
{
	ai_waypoints_x_.clear();
	ai_waypoints_y_.clear();
	for (size_t i = 0; i < settings_.N; ++i)
	{
		ai_waypoints_x_.push_back(solution.x[state_.x_.start_index_ + i]);
		ai_waypoints_y_.push_back(solution.x[state_.y_.start_index_ + i]);
	}
}
