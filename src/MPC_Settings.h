#pragma once

struct MPC_Settings
{
	size_t N;
	double dt;
	double Lf;
	double tar_v;

	double cte_cost_w;
	double v_cost_w;
	double epsi_cost_w;
	double steer_cost_w;
	double steer_delta_cost_w;
};
