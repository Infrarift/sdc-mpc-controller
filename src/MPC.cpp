#include <iostream>
#include "MPC.h"
#include "State.h"
#include "Solver.h"
#include "CostModule.h"
#include "Eigen-3.3/Eigen/QR"

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
		for (size_t i = 0, m = 5; i < m; i++)
		{
			auto state = *previous_state.states_[i];
			fg[state.start_index_ + 1] = state.val_;
		}

		for (size_t t = 0; t < settings_.N; ++t)
		{
			previous_state.SetValues(vars, t);
			next_state.SetValues(vars, t + 1);

			// Solve state for each step.
			if (t < settings_.N - 1)
				solver.Solve(fg, previous_state, next_state, coeffs, t + 2, settings_.dt);

			// Solve cost for each step.
			for (size_t i = 0, m = cost_modules_->size(); i < m; i++)
			{
				if ((*cost_modules_)[i]->max_index_ > t)
					fg[0] += (*cost_modules_)[i]->GetCost(previous_state, next_state);
			}
		}
	}
};

MPC::MPC(MPC_Settings settings)
{
	settings_ = settings;

	AddCost(&cte_cost_, settings_.cte_cost_w, settings_.N);
	AddCost(&v_cost_, settings_.v_cost_w, settings_.N);
	AddCost(&epsi_cost_, settings_.epsi_cost_w, settings_.N);
	AddCost(&steer_cost_, settings_.steer_cost_w, settings_.N - 1);
	AddCost(&steer_delta_cost_, settings_.steer_delta_cost_w, settings_.N - 2);

	state_.Initialize(settings_.N);

	n_states_ = 6;
	n_actuators_ = 2;
	n_vars_ = n_states_ * settings_.N + n_actuators_ * (settings_.N - 1);
	n_constraints_ = n_states_ * settings_.N;
}

MPC::~MPC()
{
}

vector<double> MPC::Solve(BasicState state, vector<double> pts_x, vector<double> pts_y)
{
	// Transform Co-ordinates to local space.
	Eigen::VectorXd local_wp_x(pts_x.size());
	Eigen::VectorXd local_wp_y(pts_y.size());
	CreateWayPoints(local_wp_x, local_wp_y, pts_x, pts_y, state.x, state.y, state.psi);

	// Create the co-efficients.
	auto coeffs = PolyFit(local_wp_x, local_wp_y, 3);

	// Create variable and constraint vectors.
	auto vars					= CreateVector(n_vars_, 0);
	auto vars_lowerbound		= CreateVector(n_vars_, -numeric_limits<float>::max());
	auto vars_upperbound		= CreateVector(n_vars_, numeric_limits<float>::max());
	auto constraints_lowerbound = CreateVector(n_constraints_, 0);
	auto constraints_upperbound = CreateVector(n_constraints_, 0);

	// Set variable limits.
	FillVector(vars_lowerbound, state_.delta_.start_index_, state_.a_.start_index_, -0.436);
	FillVector(vars_upperbound, state_.delta_.start_index_, state_.a_.start_index_, 0.436);
	FillVector(vars_lowerbound, state_.a_.start_index_, n_vars_, -1.0);
	FillVector(vars_upperbound, state_.a_.start_index_, n_vars_, 1.0);

	// Add the initial state constraints.
	AddInitialStateConstraints(state, coeffs, vars, constraints_lowerbound, constraints_upperbound);

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
	auto ok = true;
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Total Cost.
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// Return MPC Waypoints.
	ExtractWaypoints(solution, coeffs);

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
	for (size_t i = start_index, m = end_index; i < m; ++i)
		vector[i] = value;
}


void MPC::AddInitialStateConstraints(BasicState& state, Eigen::VectorXd coeffs, Dvector& vars1, Dvector& vars2, Dvector& vars3)
{
	state.cte = PolyEval(coeffs, 0);
	state.epsi = -atan(coeffs[1]);

	auto dt		= 0.1;
	auto p_x	= 0.0 + state.v * dt;
	auto p_y_	= 0.0;
	auto p_psi	= 0.0 + state.v * -state.delta / settings_.Lf * dt;
	auto p_v	= state.v + state.a * dt;
	auto p_cte	= state.cte + state.v * sin(state.epsi) * dt;
	auto p_epsi = state.epsi + state.v * -state.delta / settings_.Lf * dt;

	Eigen::VectorXd state_vector(6);
	state_vector << p_x , p_y_ , p_psi , p_v , p_cte , p_epsi;

	for (size_t i = 0, m = state_vector.size(); i < m; i++)
	{
		auto j = i * settings_.N;
		vars1[j] = state_vector[i];
		vars2[j] = state_vector[i];
		vars3[j] = state_vector[i];
	}
}


void MPC::ExtractWaypoints(CppAD::ipopt::solve_result<Dvector> solution, Eigen::VectorXd poly)
{
	ai_waypoints_x_.clear();
	ai_waypoints_y_.clear();

	map_waypoints_x_.clear();
	map_waypoints_y_.clear();

	for (size_t i = 0; i < settings_.N; ++i)
	{
		ai_waypoints_x_.push_back(solution.x[state_.x_.start_index_ + i]);
		ai_waypoints_y_.push_back(solution.x[state_.y_.start_index_ + i]);
	}

	for (size_t i = 1; i < 30; ++i)
	{
		auto n_x = 3 * i;
		auto n_y = PolyEval(poly, n_x);
		map_waypoints_x_.push_back(n_x);
		map_waypoints_y_.push_back(n_y);
	}
}

double MPC::PolyEval(Eigen::VectorXd coeffs, double x) const
{
	auto result = 0.0;
	for (size_t i = 0, m = coeffs.size(); i < m; i++)
		result += coeffs[i] * pow(x, i);
	return result;
}

Eigen::VectorXd MPC::PolyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (size_t i = 0, m = xvals.size(); i < m; i++)
		A(i, 0) = 1.0;

	for (size_t j = 0, m = xvals.size(); j < m; j++)
	{
		for (auto i = 0; i < order; i++)
			A(j, i + 1) = A(j, i) * xvals(j);
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

void MPC::CreateWayPoints(Eigen::VectorXd& x_wp, Eigen::VectorXd& y_wp, const vector<double>& x_mp, const vector<double>& y_mp, double x, double y, double psi) const
{
	auto cos_theta = cos(-psi);
	auto sin_theta = sin(-psi);

	for (size_t i = 0; i < x_mp.size(); i++)
	{
		auto dx = x_mp[i] - x;
		auto dy = y_mp[i] - y;

		x_wp(i) = dx * cos_theta - dy * sin_theta;
		y_wp(i) = dx * sin_theta + dy * cos_theta;
	}
}
